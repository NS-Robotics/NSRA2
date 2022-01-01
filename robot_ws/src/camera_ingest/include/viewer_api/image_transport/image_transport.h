#pragma once

#include "blockingconcurrentqueue.h"
#include "nssc_errors.h"
#include "camera_manager.h"
#include "node.h"
#include <thread>
#include "image_transport/image_transport.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define CON_THREAD_NUM 10

class NSSCImageTransport : public NSSC_ERRORS
{
    public:
        NSSCImageTransport(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager);
        ~NSSCImageTransport();

        NSSC_STATUS startPipeline();

    private:
        std::shared_ptr<cameraManager> camManager;
        std::shared_ptr<NSSC> node;

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        image_transport::Publisher left_pub;
        image_transport::Publisher right_pub;

        void transThreadFunc();
        std::thread sThread;

        std::atomic<bool> streamON{false};
};