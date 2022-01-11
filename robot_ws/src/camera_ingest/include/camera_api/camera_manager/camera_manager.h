#ifndef NSSC_CAMERA_MANAGER_
#define NSSC_CAMERA_MANAGER_

#include "camera.h"
#include "node.h"
#include "config.h"
#include "nssc_errors.h"
#include "stereo_frame.h"
#include "cam_sync.h"
#include "blockingconcurrentqueue.h"

class cameraManager : public NSSC_ERRORS
{
    public:
        cameraManager(std::shared_ptr<NSSC> &node);
        ~cameraManager();
        NSSC_STATUS init();

        NSSC_STATUS loadCameras();
        NSSC_STATUS closeCameras();

        NSSC_STATUS setExposure(float exposure_time);
        NSSC_STATUS setGain(float gain);

        stereoFrame *getFrame();
        NSSC_STATUS returnBuf(stereoFrame* stereoBuf);

    private:        
        std::shared_ptr<NSSC>   node;
        std::unique_ptr<Camera> cam1;
        std::unique_ptr<Camera> cam2;
        std::shared_ptr<CyclicBarrier> cb;

        std::thread camLeftThread;
        std::thread camRightThread;

        std::string msgCaller = "CameraManager";

        moodycamel::BlockingConcurrentQueue<stereoFrame*> emptyFrameBuf;
        std::atomic<int>  numOfEmpty{0};

        bool is_closed = false;
};

#endif //NSSC_CAMERA_MANAGER_