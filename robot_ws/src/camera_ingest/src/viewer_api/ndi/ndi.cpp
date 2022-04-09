#include "ndi.h"
#include "node.h"
#include "stereo_frame.h"
#include "frame_manager.h"
#include "nssc_errors.h"
#include <iostream>
#include <chrono>
#include <ctime>    

nssc::send::NDI::NDI(std::shared_ptr<ros::NSSC>& node, std::unique_ptr<FrameManager>* frameManager) : NSSC_ERRORS(node)
{
    this->node = node;
    this->frame_manager = frameManager;
}

nssc::send::NDI::~NDI()
{
    closeNDI();
}

nssc::NSSC_STATUS nssc::send::NDI::init()
{
    NSSC_STATUS status;

    NDIlib_initialize();

    NDIlib_send_create_t NDI_send_create_desc;
	NDI_send_create_desc.p_ndi_name = "NSSC";

    this->pNDI_send = NDIlib_send_create(&NDI_send_create_desc);
    if(!this->pNDI_send)
    {    
        status = NSSC_NDI_STATUS_SENDER_INIT_ERROR;
    } else
    {
        NDIlib_metadata_frame_t NDI_connection_type;
        NDI_connection_type.p_data = "<ndi_product long_name=\"NS Stereo Camera\" "
                                     "             short_name=\"NSSC\" "
                                     "             manufacturer=\"NSElectronics\" "
                                     "             version=\"1.000.000\" "
                                     "             session=\"default\" "
                                     "             model_name=\"S1\" "
                                     "             serial=\"1509\"/>";
        NDIlib_send_add_connection_metadata(this->pNDI_send, &NDI_connection_type);

        this->ndi_video_frame.xres = this->node->g_config.frame_config.stream_x_res;
        this->ndi_video_frame.yres = this->node->g_config.frame_config.stream_y_res;
        this->ndi_video_frame.FourCC = this->node->g_config.frame_config.FourCC;
        this->ndi_video_frame.p_data = (uint8_t *)malloc(this->node->g_config.frame_config.stream_buf_size);
        this->ndi_video_frame.line_stride_in_bytes = this->node->g_config.frame_config.ndi_line_stride;

        status = NSSC_STATUS_SUCCESS;
        this->node->printInfo(this->msg_caller, "initalized!");
    }

    this->is_closed = false;

    return status;
}

nssc::NSSC_STATUS nssc::send::NDI::startStream()
{
    NSSC_STATUS status = NSSC_STATUS_SUCCESS;

    this->stream_running = true;
    this->node->g_config.frame_config.stream_on = true;

    this->sThread = this->node->g_config.frame_config.mono_stream ? std::thread(&NDI::monoStreamThread, this) : std::thread(&NDI::stereoStreamThread, this);

    this->node->printInfo(this->msg_caller, "Stream started");

    return status;
}

nssc::NSSC_STATUS nssc::send::NDI::endStream()
{
    this->stream_running = false;
    this->node->g_config.frame_config.stream_on = false;
    this->sThread.join();
    this->node->printInfo(this->msg_caller, "Stream ended");

    return NSSC_STATUS_SUCCESS;
}

nssc::NSSC_STATUS nssc::send::NDI::closeNDI()
{
    if (this->is_closed) { return NSSC_STATUS_SUCCESS; }
    if (this->stream_running.load()) { endStream(); }

    NDIlib_send_destroy(this->pNDI_send);
    NDIlib_destroy();
    this->node->printInfo(this->msg_caller, "NDI closed");
    this->is_closed = true;
    
    return NSSC_STATUS_SUCCESS;
}

void nssc::send::NDI::stereoStreamThread()
{
    framestruct::StereoFrame* stereoFrame;

    int idx = 0;

    while(this->stream_running.load())
    {
        while(!NDIlib_send_get_no_connections(this->pNDI_send, 10000) && this->stream_running.load())
	    {
            this->node->printInfo(this->msg_caller, "No current connections, so no rendering needed (%d).");
	    }

        stereoFrame = (*this->frame_manager)->getFrame();
        if(stereoFrame == nullptr) { break; }
        
        this->ndi_video_frame.p_data = (uint8_t*) stereoFrame->stereo_buf->hImageBuf;
		NDIlib_send_send_video_async_v2(this->pNDI_send, &this->ndi_video_frame);

        (*this->frame_manager)->returnBuf(stereoFrame);

        idx = (idx == 0) ? 1 : 0;
	}
	NDIlib_send_send_video_async_v2(this->pNDI_send, NULL);
}

void nssc::send::NDI::monoStreamThread()
{
    framestruct::StereoFrame *stereoFrame[2];

    stereoFrame[0] = (*this->frame_manager)->getFrame();

    int idx = 1;

    while (this->stream_running.load())
    {
        while (!NDIlib_send_get_no_connections(this->pNDI_send, 10000) && this->stream_running.load())
        {
            this->node->printInfo(this->msg_caller, "No current connections, so no rendering needed (%d).");
        }

        stereoFrame[idx] = (*this->frame_manager)->getFrame();

        if (this->node->g_config.ingest_config.is_running)
        {
            cv::Mat sendFrame(cv::Size(this->node->g_config.frame_config.stream_x_res, this->node->g_config.frame_config.stream_y_res), CV_8UC4, stereoFrame[idx]->left_camera->rgba_buf.hImageBuf);

            cv::putText(sendFrame, "Image idx: " + std::to_string(this->node->g_config.ingest_config.current_frame_idx) + " out of: " + std::to_string(this->node->g_config.ingest_config.ingest_amount), cv::Point(25, 60), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);

            auto now = std::chrono::high_resolution_clock::now();
            auto time_left = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->node->g_config.ingest_config.sleep_timestamp);
            int countdown_i = 5000 - time_left.count();
            if (countdown_i < 0)
            {
                countdown_i = 0;
            }
            std::string countdown = "Next image in " + std::to_string(countdown_i) + " ms";

            if (countdown_i == 0 && this->node->g_config.ingest_config.image_taken)
            {
                countdown = "Image taken!";
            }

            cv::putText(sendFrame, countdown, cv::Point(1000, 60),
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);
        }

        this->ndi_video_frame.p_data = (uint8_t *)stereoFrame[idx]->right_camera->rgba_buf.hImageBuf;
        NDIlib_send_send_video_async_v2(this->pNDI_send, &this->ndi_video_frame);

        (*this->frame_manager)->returnBuf(stereoFrame[idx ^ 1]);

        idx = (idx == 0) ? 1 : 0;
    }
    NDIlib_send_send_video_async_v2(this->pNDI_send, NULL);
}