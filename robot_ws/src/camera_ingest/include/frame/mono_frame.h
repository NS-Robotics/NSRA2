#pragma once

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

class monoFrame
{
  public:
    static monoFrame *make_frame(NSSC_FRAME_TYPE type);
    void setTimestamp();
    
    virtual void convert(frameBuffer* rgbBuf) = 0;
    virtual void alloc(std::shared_ptr<NSSC>& node) = 0;

    std::shared_ptr<NSSC> node;
    frameBuffer           frameBuf;
    frameBuffer           resizeBuf;
    frameBuffer*          inputBuf;
    bool                  inputFlag = false;
};