#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include "camera/camera.h"
#include "camera/photography_settings.h"
#include "camera/device_discovery.h"
#include "camera/ins_types.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

std::atomic<bool> shutdown_requested(false);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        shutdown_requested = true;
    }
}

class TestStreamDelegate : public ins_camera::StreamDelegate {
private:
    FILE* file1_;
    FILE* file2_;
    int64_t last_timestamp = 0;
    AVCodec* codec;
    AVCodecContext* codecCtx;
    AVFrame* avFrame;
    AVPacket* pkt;
    struct SwsContext* img_convert_ctx;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

public:
    // Constructor that accepts an rclcpp::Node shared_ptr to create the publisher
    TestStreamDelegate(std::shared_ptr<rclcpp::Node> node) {
        image_pub = node->create_publisher<sensor_msgs::msg::Image>("insta_image_yuv", 10);

        file1_ = fopen("./01.h264", "wb");
        file2_ = fopen("./02.h264", "wb");

        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec) {
            std::cerr << "Codec not found\n";
            exit(1);
        }

        codecCtx = avcodec_alloc_context3(codec);
        codecCtx->flags2 |= AV_CODEC_FLAG2_FAST;
        if (!codecCtx) {
            std::cerr << "Could not allocate video codec context\n";
            exit(1);
        }

        if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
            std::cerr << "Could not open codec\n";
            exit(1);
        }

        avFrame = av_frame_alloc();
        pkt = av_packet_alloc();
    }

    ~TestStreamDelegate() {
        fclose(file1_);
        fclose(file2_);
        av_frame_free(&avFrame);
        av_packet_free(&pkt);
        avcodec_free_context(&codecCtx);
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }

    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        if (stream_index == 0) {
            pkt->data = const_cast<uint8_t*>(data);
            pkt->size = size;

            if (avcodec_send_packet(codecCtx, pkt) == 0) {
                while (avcodec_receive_frame(codecCtx, avFrame) == 0) {
                    if (!avFrame || !avFrame->data[0] || !avFrame->data[1] || !avFrame->data[2]) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid frame data");
                        continue;
                    }

                    int width = avFrame->width;
                    int height = avFrame->height;
                    int y_height = height;
                    int uv_height = height / 2;
                    int uv_width = width / 2;

                    // Allocate cv::Mat for YUV420P (no padding in the target buffer)
                    cv::Mat yuv(height * 3 / 2, width, CV_8UC1);
                    if (yuv.empty()) {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to allocate cv::Mat");
                        continue;
                    }

                    // Copy Y (luma) plane
                    for (int i = 0; i < y_height; i++) {
                        memcpy(
                            yuv.data + i * width,                            // Destination (row-by-row)
                            avFrame->data[0] + i * avFrame->linesize[0],     // Source (account for padding)
                            width                                            // Copy only the actual width
                        );
                    }

                    // Copy U (chroma) plane
                    for (int i = 0; i < uv_height; i++) {
                        memcpy(
                            yuv.data + y_height * width + i * uv_width,      // Destination (after Y plane)
                            avFrame->data[1] + i * avFrame->linesize[1],     // Source (account for padding)
                            uv_width                                         // Copy only the actual width
                        );
                    }

                    // Copy V (chroma) plane
                    for (int i = 0; i < uv_height; i++) {
                        memcpy(
                            yuv.data + y_height * width + uv_height * uv_width + i * uv_width,  // Destination
                            avFrame->data[2] + i * avFrame->linesize[2],                        // Source
                            uv_width                                                            // Copy only the actual width
                        );
                    }

                    sensor_msgs::msg::Image msg;
                    msg.header.stamp = rclcpp::Clock().now();
                    msg.header.frame_id = "camera_frame";
                    msg.height = yuv.rows;
                    msg.width = yuv.cols;
                    msg.encoding = "8UC1";
                    msg.is_bigendian = 0;
                    msg.step = yuv.cols * yuv.elemSize();
                    msg.data.assign(yuv.datastart, yuv.dataend);
                    image_pub->publish(msg);
                }
            }
        }
    }

    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {}

    void OnExposureData(const ins_camera::ExposureData& data) override {}
};

class CameraWrapper {
private:
    std::shared_ptr<ins_camera::Camera> cam;

public:
    CameraWrapper(int argc, char* argv[]) {
        rclcpp::init(argc, argv);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opened Camera");
    }

    ~CameraWrapper() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Closing Camera");
        if (this->cam) {
            this->cam->Close();
        }
    }

   int run_camera() {
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        for (int i = 0; i < list.size(); ++i) {
            auto desc = list[i];
            std::cout << "serial:" << desc.serial_number << "\t"
                << "camera type:" << int(desc.camera_type) << "\t"
                << "lens type:" << int(desc.lens_type) << std::endl;
        }

        if (list.size() <= 0) {
            std::cerr << "no device found." << std::endl;
            return -1;
        }

        this->cam = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!this->cam->Open()) {
            std::cerr << "failed to open camera" << std::endl;
            return -1;
        }
        std::cout << "http base url:" << cam->GetHttpBaseUrl() << std::endl;

        auto node = rclcpp::Node::make_shared("insta_camera_node");
        
        // Pass the node to the TestStreamDelegate
        std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>(node);
        cam->SetStreamDelegate(delegate);
        discovery.FreeDeviceDescriptors(list);
        std::cout << "Successfully opened camera..." << std::endl;
        auto camera_type = cam->GetCameraType();
        auto start = time(NULL);
        cam->SyncLocalTimeToCamera(start);
        ins_camera::LiveStreamParam param;
        // param.video_resolution = ins_camera::VideoResolution::RES_1152_1152P30;
        // param.video_bitrate = 1024 * 1024 / 1000;
        // param.enable_audio = false;
        // param.using_lrv = false;
        // param.video_resolution = ins_camera::VideoResolution::RES_720_360P30;
        // param.lrv_video_resulution = ins_camera::VideoResolution::RES_720_360P30;
        param.video_bitrate = 1024 * 1024 / 100;
        param.enable_audio = false;
        param.using_lrv = false;
        
        do {} while (!cam->StartLiveStreaming(param));
        std::cout << "successfully started live stream" << std::endl;
        rclcpp::spin(node);
        return 0;
    }
};

int main(int argc, char* argv[]) {
    // Register signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signal_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    CameraWrapper camera(argc, argv);
    int result = camera.run_camera();
        
    rclcpp::shutdown();
    return result;
}
