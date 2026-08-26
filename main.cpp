#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayPublisher.h>

// --- PHYSICAL CONFIGURATION ---
const int TEAM_NUMBER = 9015; 
const double IN_TO_M = 0.0254;

// 1. Mounting Offsets (Relative to Robot Center)
const double ROBOT_X_OFFSET = 5.0 * IN_TO_M; // Camera is 5" forward of center (Adjust this!)
const double ROBOT_Y_OFFSET = 0.0 * IN_TO_M; // Camera is centered laterally (Adjust this!)

// 2. Heights (Meters)
const double CAM_H = 0.6875 * IN_TO_M;   
const double BALL_R = 2.955 * IN_TO_M;  
const double H_DIFF = BALL_R - CAM_H;

// 3. Mounting Angles (Degrees)
const double MOUNT_PITCH = -15.0; // Tilted DOWN from floor (Adjust this!)
const double MOUNT_YAW   = 0.0; 

// 4. Logic Settings
const float CONF_THRESH = 0.70f;
const double CLUSTER_RADIUS = 0.5; // Meters

struct Ball {
    double robX, robY, yaw, dist;
    float conf;
};

inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

int main() {
    // NT4 Setup
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("ZenithVision");
    auto targetPub = table->GetDoubleArrayTopic("target").Publish(); // [X, Y, Yaw, Count]
    inst.SetServerTeam(TEAM_NUMBER);
    inst.StartClient4("ZenithPi");

    // Load Calibration
    cv::FileStorage fs("../camera_params.yaml", cv::FileStorage::READ);
    cv::Mat K, D;
    fs["camera_matrix"] >> K; fs["distortion_coefficients"] >> D;

    // NPU Init
    rknn_context ctx;
    rknn_init(&ctx, (void*)"../model.rknn", 0, 0, NULL);
    rknn_tensor_attr out_attr[3];
    for (int i = 0; i < 3; i++) {
        out_attr[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(out_attr[i]), sizeof(rknn_tensor_attr));
    }

    // Camera (960x600 YUYV)
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 600);

    cv::Mat frame, img_rgb, img_resized;
    while (true) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::resize(frame, img_resized, cv::Size(640, 640));
        cv::cvtColor(img_resized, img_rgb, cv::COLOR_BGR2RGB);

        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0; inputs[0].type = RKNN_TENSOR_UINT8; inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].size = 640*640*3; inputs[0].buf = img_rgb.data; inputs[0].pass_through = 0;
        rknn_inputs_set(ctx, 1, inputs);
        rknn_run(ctx, NULL);

        rknn_output outputs[3];
        for(int i=0; i<3; i++) outputs[i].want_float = 1;
        rknn_outputs_get(ctx, 3, outputs, NULL);

        std::vector<Ball> detections;

        // Post-Processing (3 Layers)
        for (int i = 0; i < 3; i++) {
            float* buffer = (float*)outputs[i].buf;
            int H = out_attr[i].dims[2]; int W = out_attr[i].dims[3];
            int grid_size = H * W;
            float stride = 640.0f / H;

            for (int j = 0; j < grid_size; j++) {
                float conf = sigmoid(buffer[4 * grid_size + j]); // Fuel class
                if (conf > CONF_THRESH) {
                    int row = j / W; int col = j % W;
                    float cx = (buffer[0*grid_size + j] * 2.0f - 0.5f + col) * stride;
                    float cy = (buffer[1*grid_size + j] * 2.0f - 0.5f + row) * stride;

                    // MATH: Transform to Physical Angles
                    cv::Point2f raw_px(cx * 960/640, cy * 600/640);
                    std::vector<cv::Point2f> dst;
                    cv::undistortPoints(std::vector<cv::Point2f>{raw_px}, dst, K, D);
                    
                    double yaw = atan(dst[0].x) * 180.0 / M_PI;
                    double pitch = -atan(dst[0].y) * 180.0 / M_PI;

                    // Distance Calculation (Angled)
                    double total_pitch_rad = (MOUNT_PITCH + pitch) * M_PI / 180.0;
                    double dist = std::abs(H_DIFF / tan(total_pitch_rad));

                    // Robot-Centric Translation
                    double rx = ROBOT_X_OFFSET + (dist * cos(yaw * M_PI / 180.0));
                    double ry = ROBOT_Y_OFFSET - (dist * sin(yaw * M_PI / 180.0));

                    detections.push_back({rx, ry, yaw, dist, conf});
                }
            }
        }

        // Clustering & Publishing
        if (!detections.empty()) {
            // Group balls near the most confident detection
            std::sort(detections.begin(), detections.end(), [](Ball a, Ball b){return a.conf > b.conf;});
            Ball best = detections[0];
            double cX = 0, cY = 0, count = 0;
            for(auto &b :
