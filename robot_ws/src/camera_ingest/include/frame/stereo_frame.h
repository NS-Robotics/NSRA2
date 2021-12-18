#ifndef NSSC_FRAME_STEREO_FRAME_
#define NSSC_FRAME_STEREO_FRAME_

#include "frame_struct.h"
#include "mono_frame.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>

class stereoFrame
{
  public:
    static stereoFrame *make_frame(NSSC_FRAME_TYPE type);
    
    virtual void convert(monoFrame* leftCamera, monoFrame* rightCamera) = 0;
    virtual void alloc(std::shared_ptr<NSSC>& node) = 0;

    std::shared_ptr<NSSC> node;
    frameBuffer   stereoBuf;
    monoFrame*    leftCamera;
    monoFrame*    rightCamera;
    bool          inputFlag = false;
    int           timedif;
};

#endif  //NSSC_FRAME_STEREO_FRAME_