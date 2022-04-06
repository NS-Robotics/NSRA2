#ifndef NSSC_CAMERA_MANAGER_
#define NSSC_CAMERA_MANAGER_

#include "camera.h"
#include "node.h"
#include "config.h"
#include "nssc_errors.h"
#include "stereo_frame.h"
#include "cam_sync.h"
#include "blockingconcurrentqueue.h"

namespace nssc
{
    namespace ingest
    {
        class CameraManager : public NSSC_ERRORS
        {
            public:
                CameraManager(std::shared_ptr<ros::NSSC> &node);
                ~CameraManager();
                NSSC_STATUS init();

                NSSC_STATUS loadCameras();
                NSSC_STATUS closeCameras();

                NSSC_STATUS setExposure(float exposure_time);
                NSSC_STATUS setGain(float gain);

                framestruct::StereoFrame *getFrame();
                NSSC_STATUS returnBuf(framestruct::StereoFrame* stereoBuf);

            private:
                std::shared_ptr<ros::NSSC>   node;
                std::unique_ptr<Camera> cam1;
                std::unique_ptr<Camera> cam2;
                std::shared_ptr<CyclicBarrier> cb;

                std::string msg_caller = "CameraManager";

                moodycamel::BlockingConcurrentQueue<framestruct::StereoFrame*> buf_empty;
                std::atomic<int>  num_empty{0};

                bool is_closed = false;
        };
    }
}

#endif //NSSC_CAMERA_MANAGER_