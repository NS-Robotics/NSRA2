#pragma once

#include "node.h"
#include "config.h"
#include <Processing.NDI.Advanced.h>
#include "nssc_errors.h"
#include "camera_manager.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "stereo_frame.h"

class NDI : public NSSC_ERRORS
{
public:
    NDI(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager);
    ~NDI();

    NSSC_STATUS init();

    NSSC_STATUS startStream();
    NSSC_STATUS endStream();

private:
    globalConfig g_config;
    
    std::shared_ptr<NSSC>           node;
    std::shared_ptr<cameraManager>  camManager;
    NDIlib_send_instance_t          pNDI_send;
    NDIlib_video_frame_v2_t         NDI_video_frame;
    
    std::string msgCaller = "NDI";

    void streamThread();

    std::atomic<bool> streamON{false};

    std::thread sThread;
};