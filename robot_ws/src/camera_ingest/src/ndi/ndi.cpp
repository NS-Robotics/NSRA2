#include "ndi.h"
#include <iostream>
#include <chrono>
#include <ctime>    

NDI::NDI(std::shared_ptr<NSSC>& node, std::shared_ptr<cameraManager>& camManager) : NSSC_ERRORS(node)
{
    this->node = node;
    this->camManager = camManager;
}

NDI::~NDI()
{
    closeNDI();
}

NSSC_STATUS NDI::init()
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

        this->NDI_video_frame.xres = this->node->g_config.frameConfig.stream_x_res;
        this->NDI_video_frame.yres = this->node->g_config.frameConfig.stream_y_res;
        this->NDI_video_frame.FourCC = this->node->g_config.frameConfig.FourCC;
        this->NDI_video_frame.p_data = (uint8_t *)malloc(this->node->g_config.frameConfig.stream_buf_size);
        this->NDI_video_frame.line_stride_in_bytes = this->node->g_config.frameConfig.ndi_line_stride;

        status = NSSC_STATUS_SUCCESS;
        this->node->printInfo(this->msgCaller, "initalized!");
    }

    this->is_closed = false;

    return status;
}

NSSC_STATUS NDI::startStream()
{
    NSSC_STATUS status = NSSC_STATUS_SUCCESS;

    this->streamON = true;

    this->sThread = this->node->g_config.frameConfig.mono_stream ? std::thread(&NDI::monoStreamThread, this) : std::thread(&NDI::stereoStreamThread, this);

    this->node->printInfo(this->msgCaller, "Stream started");

    return status;
}

NSSC_STATUS NDI::endStream()
{
    this->streamON = false;
    this->sThread.join();
    this->node->printInfo(this->msgCaller, "Stream ended");

    return NSSC_STATUS_SUCCESS;
}

NSSC_STATUS NDI::closeNDI()
{
    if(this->is_closed) { return NSSC_STATUS_SUCCESS; }

    NDIlib_send_destroy(this->pNDI_send);
    NDIlib_destroy();
    this->node->printInfo(this->msgCaller, "NDI closed");
    this->is_closed = true;
    
    return NSSC_STATUS_SUCCESS;
}

void NDI::stereoStreamThread()
{
    stereoFrame* stereoFrame;

    int idx = 0;

    while(this->streamON.load())
    {
        while(!NDIlib_send_get_no_connections(this->pNDI_send, 10000) && this->streamON.load())
	    {
            this->node->printInfo(this->msgCaller, "No current connections, so no rendering needed (%d).");
	    }

        stereoFrame = this->camManager->getFrame(true, this->node->g_config.frameConfig.resize_frame);

        if(this->node->g_config.ingestConfig.is_running)
        {
            cv::Mat sendFrame(cv::Size(this->node->g_config.frameConfig.stream_x_res, this->node->g_config.frameConfig.stream_y_res), CV_8UC4, stereoFrame->stereoBuf->hImageBuf);

            cv::putText(sendFrame, "Image idx: " + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + " out of: " + std::to_string(this->node->g_config.ingestConfig.ingest_amount), cv::Point(25, 60), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);

            auto now = std::chrono::high_resolution_clock::now();
            auto time_left = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->node->g_config.ingestConfig.sleep_timestamp);
            int countdown_i = 5000 - time_left.count();
            if(countdown_i < 0)
            {
                countdown_i = 0;
            }
            std::string countdown = "Next image in " + std::to_string(countdown_i) + " ms";

            if(countdown_i == 0 && this->node->g_config.ingestConfig.image_taken)
            {
                countdown = "Image taken!";
            }

            cv::putText(sendFrame, countdown, cv::Point(1000, 60),
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);
        }
        
        this->NDI_video_frame.p_data = (uint8_t*) stereoFrame->stereoBuf->hImageBuf;
		NDIlib_send_send_video_async_v2(this->pNDI_send, &this->NDI_video_frame);

        this->camManager->returnBuf(stereoFrame);

        idx = (idx == 0) ? 1 : 0;
	}
	NDIlib_send_send_video_async_v2(this->pNDI_send, NULL);
}

void NDI::monoStreamThread()
{
    stereoFrame *stereoFrame;

    int idx = 0;

    while (this->streamON.load())
    {
        while (!NDIlib_send_get_no_connections(this->pNDI_send, 10000) && this->streamON.load())
        {
            this->node->printInfo(this->msgCaller, "No current connections, so no rendering needed (%d).");
        }

        stereoFrame = this->camManager->getFrame();

        if (this->node->g_config.ingestConfig.is_running)
        {
            cv::Mat sendFrame(cv::Size(this->node->g_config.frameConfig.stream_x_res, this->node->g_config.frameConfig.stream_y_res), CV_8UC4, stereoFrame->leftCamera->frameBuf.hImageBuf);

            cv::putText(sendFrame, "Image idx: " + std::to_string(this->node->g_config.ingestConfig.current_frame_idx) + " out of: " + std::to_string(this->node->g_config.ingestConfig.ingest_amount), cv::Point(25, 60), //top-left position
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);

            auto now = std::chrono::high_resolution_clock::now();
            auto time_left = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->node->g_config.ingestConfig.sleep_timestamp);
            int countdown_i = 5000 - time_left.count();
            if (countdown_i < 0)
            {
                countdown_i = 0;
            }
            std::string countdown = "Next image in " + std::to_string(countdown_i) + " ms";

            if (countdown_i == 0 && this->node->g_config.ingestConfig.image_taken)
            {
                countdown = "Image taken!";
            }

            cv::putText(sendFrame, countdown, cv::Point(1000, 60),
                        cv::FONT_HERSHEY_DUPLEX,
                        2.0,
                        cv::Scalar(254, 0, 0),
                        2);
        }

        this->NDI_video_frame.p_data = (uint8_t *)stereoFrame->leftCamera->frameBuf.hImageBuf;
        NDIlib_send_send_video_async_v2(this->pNDI_send, &this->NDI_video_frame);

        this->camManager->returnBuf(stereoFrame);

        idx = (idx == 0) ? 1 : 0;
    }
    NDIlib_send_send_video_async_v2(this->pNDI_send, NULL);
}