#pragma once

#include <Processing.NDI.Advanced.h>

typedef int NSSC_BUF_SIZE;

typedef enum NSSC_FRAME_TYPES
{

    NSSC_FRAME_RGBA         = 0,
    NSSC_FRAME_I420         = 1,
    NSSC_FRAME_UYVY         = 2

} NSSC_FRAME_TYPE;

struct globalConfig
{
public:
    NSSC_FRAME_TYPE g_type  = NSSC_FRAME_RGBA;
    short cam_x_res         = 3088;
    short cam_y_res         = 2064;

    float cam_exposure_time = 70000.0;
    float cam_gain          = 15.0;

    bool resize_frame       = true;

    short mono_x_res        = resize_frame ? 1920 : cam_x_res;
    short mono_y_res        = resize_frame ? 1080 : cam_y_res;

    short stream_x_res      = mono_x_res * 2;
    short stream_y_res      = mono_y_res;

    NDIlib_FourCC_video_type_e FourCC;

    NSSC_BUF_SIZE stereo_buf_size;
    NSSC_BUF_SIZE mono_buf_size;
    NSSC_BUF_SIZE ndi_line_stride;

    globalConfig()
    {
        switch(g_type)
        {
            case NSSC_FRAME_RGBA:
                stereo_buf_size = mono_x_res * mono_y_res * 4 * 2;
                mono_buf_size = mono_x_res * mono_y_res * 4;
                ndi_line_stride = stream_x_res * 4;
                FourCC = NDIlib_FourCC_type_RGBX;
                break;
            
            case NSSC_FRAME_I420:
                stereo_buf_size = mono_x_res * std::round(mono_y_res * 1.5) * 2;
                mono_buf_size = mono_x_res * std::round(mono_y_res * 1.5);
                ndi_line_stride = mono_x_res;
                FourCC = NDIlib_FourCC_type_I420;
                break;
            case NSSC_FRAME_UYVY:
                stereo_buf_size = mono_x_res * mono_y_res * 2 * 2;
                mono_buf_size = mono_x_res * mono_y_res * 2;
                ndi_line_stride = stream_x_res * 2;
                FourCC = NDIlib_FourCC_type_UYVY;
                break;
        }
    }

};
