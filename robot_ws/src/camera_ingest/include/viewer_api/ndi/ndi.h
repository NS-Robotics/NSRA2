#ifndef NSSC_NDI_
#define NSSC_NDI_

#include "node.h"
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

    NSSC_STATUS closeNDI();

private:
    std::shared_ptr<NSSC>           node;
    std::shared_ptr<cameraManager>  camManager;
    NDIlib_send_instance_t          pNDI_send;
    NDIlib_video_frame_v2_t         NDI_video_frame;
    
    std::string msgCaller = "NDI";

    void stereoStreamThread();
    void monoStreamThread();

    std::atomic<bool> streamON{false};

    std::thread sThread;

    bool is_closed = false;
};

#endif  //NSSC_NDI_