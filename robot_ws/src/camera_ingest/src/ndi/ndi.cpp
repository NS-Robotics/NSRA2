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
    endStream();
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
                                     "             serial=\"lolhahatest\"/>";
        NDIlib_send_add_connection_metadata(this->pNDI_send, &NDI_connection_type);

        this->NDI_video_frame.xres = this->node->g_config.frameConfig.stream_x_res;
        this->NDI_video_frame.yres = this->node->g_config.frameConfig.stream_y_res;
        this->NDI_video_frame.FourCC = this->node->g_config.frameConfig.FourCC;
        this->NDI_video_frame.p_data = (uint8_t *)malloc(this->node->g_config.frameConfig.stereo_buf_size);
        this->NDI_video_frame.line_stride_in_bytes = this->node->g_config.frameConfig.ndi_line_stride;

        status = NSSC_STATUS_SUCCESS;
        this->node->printInfo(this->msgCaller, "initalized");
    }

    return status;
}

NSSC_STATUS NDI::startStream()
{
    NSSC_STATUS status = NSSC_STATUS_SUCCESS;

    this->streamON = true;

    this->sThread = std::thread(&NDI::streamThread, this);

    this->node->printInfo(this->msgCaller, "Thread started!");

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
    NDIlib_send_destroy(this->pNDI_send);
    NDIlib_destroy();
    this->node->printInfo(this->msgCaller, "NDI closed");

    return NSSC_STATUS_SUCCESS;
}

void NDI::streamThread()
{
    stereoFrame* stereoFrame;

    int idx = 0;

    while(this->streamON.load())
    {
        while(!NDIlib_send_get_no_connections(this->pNDI_send, 10000) && this->streamON.load())
	    {
            this->node->printInfo(this->msgCaller, "No current connections, so no rendering needed (%d).");
	    }

        auto start0 = std::chrono::high_resolution_clock::now();
        stereoFrame = this->camManager->getFrame();
        auto stop0 = std::chrono::high_resolution_clock::now();
        
        auto start2 = std::chrono::high_resolution_clock::now();

        this->NDI_video_frame.p_data = (uint8_t*) stereoFrame->stereoBuf.hImageBuf;
		NDIlib_send_send_video_async_v2(this->pNDI_send, &this->NDI_video_frame);

        this->camManager->returnBuf(stereoFrame);

        idx = (idx == 0) ? 1 : 0;

        auto stop2 = std::chrono::high_resolution_clock::now();

        auto duration0 = std::chrono::duration_cast<std::chrono::microseconds>(stop0 - start0);
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - start2);

        auto total_dur = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - stereoFrame->leftCamera->frameBuf.timestamp);

        //this->node->printInfo(this->msgCaller, "Frame diff: " + std::to_string(stereoFrame->timedif));

        //this->node->printInfo(this->msgCaller, "Frame timing: getFrame - " + std::to_string(duration0.count()) + " | sendFrame - " + std::to_string(duration2.count()) + " | total - " + std::to_string(duration0.count() + duration2.count()));
        //this->node->printInfo(this->msgCaller, "Frame timing: frame total - " + std::to_string(total_dur.count()) + " | total - " + std::to_string(duration0.count() + duration2.count()));
	}
	NDIlib_send_send_video_async_v2(this->pNDI_send, NULL);
}