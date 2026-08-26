#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"

// WPILib NetworkTables
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayPublisher.h>

// --- PHYSICAL CONFIGURATION (Inches -> Meters) ---
const double IN_TO_M = 0.0254;
const double CAM_H = 0.6875 * IN_TO_M;   
const double BALL_R = 2.955 * IN_TO_M;  
const double H_DIFF = BALL_R - CAM_H;

// --- MOUNTING ANGLES (Degrees) ---
const double MOUNT_PITCH = 15.0; 
const double MOUNT_YAW   = 0.0;

// --- ROBOT OFFSETS (Meters) ---
// Coordinates of the lens relative to the center of the robot
const double ROBOT_X_OFFSET = 5.0 * IN_TO_M;
const double ROBOT_Y_OFFSET = 0.0 * IN_TO_M;

// --- VISION SETTINGS ---
const float CONF_THRESH = 0.75f;
const double CLUSTER_RADIUS = 0.5; // Meters to group balls together

// Global for MJPEG Stream
std::mutex mtx;
cv::Mat stream_frame;

inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

struct Ball {
    double x, y, dist, yaw;
    float conf;
    cv::Rect box;
};

// Simple MJPEG Server on Port 8080, will be done later.
void start_web_server() {
    std::cout << "Web server logic would go here (Port 8080)" << std::endl;
}

int main() {
    // 1. Setup NetworkTables
    auto nt_inst = nt::NetworkTableInstance::GetDefault();
    auto table = nt_inst.GetTable("ZenithVision");
    auto targetPub = table->GetDoubleArrayTopic("target").Publish(); // [x, y, yaw, is_cluster]
    nt_inst.SetServerTeam(9015);
    nt_inst.StartClient4("ZenithPi");

    // 2. Load Calibration
    cv::FileStorage fs("../camera_params.yaml", cv::FileStorage::READ);
    cv::Mat K, D;
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;
    fs.release();

    // 3. Initialize NPU (YOLOv8)
    rknn_context ctx;
    rknn_init(&ctx, (void*)"../model.rknn", 0, 0, NULL);
    rknn_tensor_attr out_attr[3];
    for (int i = 0; i < 3; i++) {
        out_attr[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(out_attr[i]), sizeof(rknn_tensor_attr));
    }

    // 4. Camera (960x600 YUYV)
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

        // NPU Inference
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0; inputs[0].type = RKNN_TENSOR_UINT8; inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].size = 640*640*3; inputs[0].buf = img_rgb.data; inputs[0].pass_through = 0;
        rknn_inputs_set(ctx, 1, inputs);
        rknn_run(ctx, NULL);

        rknn_output outputs[3];
        for(int i=0; i<3; i++) outputs[i].want_float = 1;
        rknn_outputs_get(ctx, 3, outputs, NULL);

        std::vector<Ball> detected_balls;

        // Post-Processing
        for (int i = 0; i < 3; i++) {
            float* buffer = (float*)outputs[i].buf;
            int H = out_attr[i].dims[2];
            int grid_size = H * out_attr[i].dims[3];
            float stride = 640.0f / H;

            for (int j = 0; j < grid_size; j++) {
                float fuel_score = sigmoid(buffer[4 * grid_size + j]); // Class 0
                if (fuel_score > CONF_THRESH) {
                    int row = j / H; int col = j % H;
                    float cx_npu = (buffer[0*grid_size + j] * 2.0f - 0.5f + col) * stride;
                    float cy_npu = (buffer[1*grid_size + j] * 2.0f - 0.5f + row) * stride;

                    // --- THE MATH ---
                    cv::Point2f raw_px(cx_npu * 960/640, cy_npu * 600/640);
                    std::vector<cv::Point2f> dst;
                    cv::undistortPoints(std::vector<cv::Point2f>{raw_px}, dst, K, D);
                    
                    double yaw_to_ball = atan(dst[0].x) * 180.0 / M_PI;
                    double pitch_to_ball = -atan(dst[0].y) * 180.0 / M_PI;

                    // Trig Distance (Corrected for Angle)
                    double total_pitch_rad = (MOUNT_PITCH + pitch_to_ball) * M_PI / 180.0;
                    double dist_to_ball = std::abs(H_DIFF / tan(total_pitch_rad));

                    // Coordinate Transform to Robot-Centric
                    double final_x = ROBOT_X_OFFSET + (dist_to_ball * cos(yaw_to_ball * M_PI / 180.0));
                    double final_y = ROBOT_Y_OFFSET - (dist_to_ball * sin(yaw_to_ball * M_PI / 180.0));

                    detected_balls.push_back({final_x, final_y, dist_to_ball, yaw_to_ball, fuel_score});
                }
            }
        }

        // CLUSTERING
        if (!detected_balls.empty()) {
            // Find the cluster center (Simplifed for FRC: Group balls near the most confident ball)
            Ball best = detected_balls[0];
            double clusterX = 0, clusterY = 0, count = 0;
            for(auto &b : detected_balls) {
                if (std::hypot(b.x - best.x, b.y - best.y) < CLUSTER_RADIUS) {
                    clusterX += b.x; clusterY += b.y; count++;
                }
            }
            // Publish the average center of the pile
            targetPub.Set({clusterX/count, clusterY/count, best.yaw, count});
        }

        rknn_outputs_release(ctx, 3, outputs);
    }
    return 0;
}
