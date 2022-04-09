#ifndef NSSC_FRAME_STEREO_FRAME_
#define NSSC_FRAME_STEREO_FRAME_

#include "frame_struct.h"
#include "mono_frame.h"
#include "node.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>

namespace nssc
{
    namespace framestruct
    {
        class StereoFrame
        {
        public:
            static StereoFrame *makeFrame(NSSC_FRAME_TYPE type);

            virtual void convert(MonoFrame *leftCamera, MonoFrame *rightCamera) = 0;

            virtual void alloc(std::shared_ptr<ros::NSSC> &node) = 0;

            virtual void process(bool resize) = 0;

            std::shared_ptr<ros::NSSC> node;
            std::string msg_caller = "Stereo Frame";

            FrameBuffer *stereo_buf;
            MonoFrame *left_camera;
            MonoFrame *right_camera;

            FrameBuffer concatenate_buf;
            FrameBuffer resize_buf;

            int timedif;
        };
    }
}

#endif  //NSSC_FRAME_STEREO_FRAME_