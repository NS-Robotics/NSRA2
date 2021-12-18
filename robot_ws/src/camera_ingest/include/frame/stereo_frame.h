#pragma once
#include "frame_struct.h"
#include "mono_frame.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>

class stereoFrame
{
  public:
    static stereoFrame *make_frame(NSSC_FRAME_TYPE type, std::shared_ptr<NSSC>& node);
    
    virtual void convert(monoFrame* leftCamera, monoFrame* rightCamera) = 0;
    virtual void alloc() = 0;

    std::shared_ptr<NSSC> node;
    frameBuffer   stereoBuf;
    monoFrame*    leftCamera;
    monoFrame*    rightCamera;
    bool          inputFlag = false;
    int           timedif;
};