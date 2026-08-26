// goes on OrangePi 5

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleArrayPublisher.h>
#include <networktables/IntegerPublisher.h>

// --- ROBOT CONFIG (METERS) ---
const double CAM_HEIGHT = 0.1;   // will be determined later
const double FUEL_RADIUS = 0.07505;  // 2.955 inches to meters
const double H_DIFF = FUEL_RADIUS - CAM_HEIGHT;
const double CLUSTER_GAP = 0.4;      // Meters (approx 15 inches)

struct Ball {
    double x, y, yaw, dist;
    float conf;
};

inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }

int main() {
    // 1. NETWORK TABLES SETUP
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("ZenithVision");
    auto bestBallPub = table->GetDoubleArrayTopic("bestBall").Publish();
    auto clusterPub = table->GetDoubleArrayTopic("cluster").Publish();
    auto countPub = table->GetIntegerTopic("ballCount").Publish();
    inst.SetServerTeam(9015);
    inst.StartClient4("ZenithPi");

    // 2. VISION ASSETS
    cv::FileStorage fs("../camera_params.yaml", cv::FileStorage::READ);
    cv::Mat K, D;
    fs["camera_matrix"] >> K; fs["distortion_coefficients"] >> D;

    rknn_context ctx;
    if (rknn_init(&ctx, (void*)"../model.rknn", 0, 0, NULL) < 0) return -1;

    rknn_tensor_attr out_attr[3];
    for (int i = 0; i < 3; i++) {
        out_attr[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(out_attr[i]), sizeof(rknn_tensor_attr));
    }

    // 3. CAMERA
    cv::VideoCapture cap(0, cv::CAP_V4L2);
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

        std::vector<Ball> detected_balls;

        // --- THE EXTRACTION LOGIC ---
        for (int i = 0; i < 3; i++) {
            float* buffer = (float*)outputs[i].buf;
            int H = out_attr[i].dims[2];
            int W = out_attr[i].dims[3];
            int grid_size = H * W;
            float stride = 640.0f / H;

            for (int j = 0; j < grid_size; j++) {
                // YOLOv8 logic: Class 0 is Fuel. 
                // Channels: 0,1,2,3 are box | 4 is Class 0 | 5 is Class 1
                float fuel_score = sigmoid(buffer[4 * grid_size + j]);

                if (fuel_score > 0.75f) {
                    int row = j / W; int col = j % W;
                    
                    // Get Box Center
                    float cx_npu = (buffer[0 * grid_size + j] * 2.0f - 0.5f + col) * stride;
                    float cy_npu = (buffer[1 * grid_size + j] * 2.0f - 0.5f + row) * stride;

                    // Convert to Undistorted Angles
                    cv::Point2f raw_px(cx_npu * 960/640, cy_npu * 600/640);
                    std::vector<cv::Point2f> dst;
                    cv::undistortPoints(std::vector<cv::Point2f>{raw_px}, dst, K, D);
                    
                    double yaw = atan(dst[0].x);
                    double pitch = -atan(dst[0].y);
                    double dist = std::abs(H_DIFF / tan(pitch));

                    // Store as Robot-Relative Meters
                    detected_balls.push_back({ dist * cos(yaw), -dist * sin(yaw), yaw, dist, fuel_score });
                }
            }
        }

        // --- CLUSTERING LOGIC ---
        if (!detected_balls.empty()) {
            std::vector<std::vector<Ball>> clusters;
            for (auto& b : detected_balls) {
                bool added = false;
                for (auto& cluster : clusters) {
                    double d = std::hypot(b.x - cluster[0].x, b.y - cluster[0].y);
                    if (d < CLUSTER_GAP) {
                        cluster.push_back(b);
                        added = true;
                        break;
                    }
                }
                if (!added) clusters.push_back({b});
            }

            // Find biggest cluster
            auto best_cluster = std::max_element(clusters.begin(), clusters.end(), 
                [](const std::vector<Ball>& a, const std::vector<Ball>& b) { return a.size() < b.size(); });

            double cX = 0, cY = 0;
            for (auto& b : *best_cluster) { cX += b.x; cY += b.y; }
            cX /= best_cluster->size(); cY /= best_cluster->size();

            // Send to Robot
            bestBallPub.Set({detected_balls[0].x, detected_balls[0].y, detected_balls[0].yaw});
            clusterPub.Set({cX, cY, (double)best_cluster->size()});
            countPub.Set((int64_t)detected_balls.size());
        } else {
            countPub.Set(0);
        }

        rknn_outputs_release(ctx, 3, outputs);
    }
    return 0;
}
