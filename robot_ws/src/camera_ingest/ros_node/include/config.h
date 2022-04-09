#ifndef NSSC_CONFIG_
#define NSSC_CONFIG_

#include <Processing.NDI.Advanced.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>

typedef int NSSC_BUF_SIZE;

typedef enum NSSC_FRAME_TYPES
{

    NSSC_FRAME_RGBA         = 0,
    NSSC_FRAME_I420         = 1,
    NSSC_FRAME_UYVY         = 2

} NSSC_FRAME_TYPE;

typedef enum NSSC_NDI_SEND_TYPES
{
    NDI_SEND_RAW            = 0,
    NDI_SEND_TRIANGULATION  = 1,
    NDI_SEND_INGEST         = 2,
    NDI_SEND_CALIBRATION    = 3,
} NSSC_NDI_SEND;

struct ColorFilterParams
{
public:
    int low_H = 40;
    int low_S = 94;
    int low_V = 23;
    int high_H = 133;
    int high_S = 218;
    int high_V = 160;
    int dilation_element = 0;
    int dilation_size = 2;
    bool enable_detection = true;
    bool enable_ndi = true;
};

struct frame_config
{
public:
    //Frame
    NSSC_FRAME_TYPE g_type = NSSC_FRAME_RGBA;
    bool resize_frame = false;

    //Camera
    const short cam_x_res = 3088;
    const short cam_y_res = 2064;

    float cam_exposure_time = 50000.0; //microseconds
    float cam_gain = 15.0;

    int max_frame_time_diff = 5000; //microseconds

    //mono
    short mono_x_res;
    short mono_y_res;
    NSSC_BUF_SIZE mono_buf_size;

    //stereo
    short stereo_x_res;
    short stereo_y_res;
    NSSC_BUF_SIZE stereo_buf_size;

    //stereo resized
    const short resize_x_res = 1920 * 2;
    const short resize_y_res = 1080;
    NSSC_BUF_SIZE resize_buf_size;

    //NDI
    bool mono_stream = false;
    bool stream_on = false;
    short stream_x_res;
    short stream_y_res;
    NSSC_BUF_SIZE stream_buf_size;

    NDIlib_FourCC_video_type_e FourCC;    
    int ndi_line_stride;

    frame_config()
    {
        calculate_params();
    }

    void calculate_params()
    {
        switch (g_type)
        {
        case NSSC_FRAME_RGBA:
            mono_x_res = cam_x_res;
            mono_y_res = cam_y_res;
            mono_buf_size = mono_x_res * mono_y_res * 4;

            stereo_x_res = 2 * mono_x_res;
            stereo_y_res = mono_y_res;
            stereo_buf_size = stereo_x_res * stereo_y_res * 4;

            resize_buf_size = resize_x_res * resize_y_res * 4;
            
            if(mono_stream)
            {
                stream_x_res = mono_x_res;
                stream_y_res = mono_y_res;
                stream_buf_size = mono_buf_size;
            }
            else
            {
                stream_x_res = resize_frame ? resize_x_res : stereo_x_res;
                stream_y_res = resize_frame ? resize_y_res : stereo_y_res;
                stream_buf_size = resize_frame ? resize_buf_size : stereo_buf_size;
            }

            ndi_line_stride = stream_x_res * 4;
            FourCC = NDIlib_FourCC_type_RGBX;
            break;

        case NSSC_FRAME_I420:
            mono_x_res = cam_x_res;
            mono_y_res = cam_y_res;
            mono_buf_size = mono_x_res * std::round(mono_y_res * 1.5);

            stereo_x_res = 2 * mono_x_res;
            stereo_y_res = mono_y_res;
            stereo_buf_size = mono_x_res * std::round(mono_y_res * 1.5) * 2;

            resize_buf_size = resize_x_res * resize_y_res * 4;

            stream_x_res = resize_frame ? resize_x_res : stereo_x_res;
            stream_y_res = resize_frame ? resize_y_res : stereo_y_res;
            stream_buf_size = resize_frame ? resize_buf_size : stereo_buf_size;

            ndi_line_stride = mono_x_res;
            FourCC = NDIlib_FourCC_type_I420;
            break;

        case NSSC_FRAME_UYVY:
            mono_x_res = cam_x_res;
            mono_y_res = cam_y_res;
            mono_buf_size = mono_x_res * mono_y_res * 2;

            stereo_x_res = 2 * mono_x_res;
            stereo_y_res = mono_y_res;
            stereo_buf_size = stereo_x_res * mono_y_res * 2;

            resize_buf_size = resize_x_res * resize_y_res * 4;

            stream_x_res = resize_frame ? resize_x_res : stereo_x_res;
            stream_y_res = resize_frame ? resize_y_res : stereo_y_res;
            stream_buf_size = resize_frame ? resize_buf_size : stereo_buf_size;

            ndi_line_stride = stream_x_res * 2;
            FourCC = NDIlib_FourCC_type_UYVY;
            break;
        }
    }
};

struct ingest_config
{
public:
    short max_frame_time_diff = 1000; //microseconds
    int wait_duration = 5000;         //milliseconds
    bool is_running = false;
    const char *left_img_name = "img_right_";
    const char *right_img_name = "img_left_";

    short current_frame_idx = 0;
    int ingest_amount = 0;
    char *set_name = "N/A";
    bool image_taken = false;
    std::chrono::high_resolution_clock::time_point sleep_timestamp;
};

struct calib_config
{
public:
    bool is_running   = false;
    char *set_name    = "N/A";
    int board_width   = 10;
    int board_height  = 7;
    float square_size = 30.5; //millimeters
};

struct triangulation_config
{
public:
    short max_origin_frame_time_diff = 1000; //microseconds
    std::vector<int> origin_ids = { 1, 2, 3 };
    char const *standard_config_file = "set2";
    ColorFilterParams color_filter_params;
};

struct globalConfig : public frame_config, public ingest_config, public calib_config, public triangulation_config
{
public:
    std::string package_name = "camera_ingest";
    std::string share_dir;

    frame_config frameConfig;
    ingest_config ingestConfig;
    calib_config calibConfig;
    triangulation_config triangulationConfig;

    globalConfig()
    {
        share_dir = ament_index_cpp::get_package_share_directory(package_name);
    }
};

#endif  //NSSC_CONFIG_