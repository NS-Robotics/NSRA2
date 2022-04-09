#ifndef NSSC_FRAME_MONO_FRAME_
#define NSSC_FRAME_MONO_FRAME_

#include "frame_struct.h"
#include "nssc_errors.h"
#include "node.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <cuda_runtime_api.h>
#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <cuda.h>
#include <cuda_runtime.h>

#include <nppi.h>
#include <npp.h>

namespace nssc
{
    namespace framestruct
    {
        class MonoFrame
        {
        public:
          static MonoFrame *makeFrame(NSSC_FRAME_TYPE type);
          void setTimestamp();

          virtual void convert(FrameBuffer *rgbBuf) = 0;
          virtual void alloc(std::shared_ptr<ros::NSSC> &node, int id) = 0;

          std::shared_ptr<ros::NSSC> node;
          std::string msg_caller = "Mono Frame";
          FrameBuffer rgba_buf;
          FrameBuffer rgb_buf;
          bool inputFlag = false;
        };
    }
}

#endif //NSSC_FRAME_MONO_FRAME_