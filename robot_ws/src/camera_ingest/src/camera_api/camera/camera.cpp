#include "camera.h"
#include "node.h"
#include "cam_sync.h"
#include "mono_frame.h"
#include "frame_struct.h"
#include "nssc_errors.h"
#include <chrono>

nssc::ingest::Camera::Camera(std::shared_ptr<nssc::ros::NSSC>& node, std::shared_ptr<CyclicBarrier>& cb) : NSSC_ERRORS(node)
{
    this->node = node;
    this->cb = cb;
}

nssc::ingest::Camera::~Camera()
{
    closeCamera();
}

nssc::NSSC_STATUS nssc::ingest::Camera::init()
{
    return GXInitLib();
}

nssc::NSSC_STATUS nssc::ingest::Camera::_printDeviceInfo()
{
    //RCLCPP_INFO(this->node->get_logger(), "***********************************************");
    //printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    NSSC_STATUS emStatus = NSSC_STATUS_SUCCESS;

    emStatus = GXGetStringLength(this->h_device, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszVendorName = new char[nSize];

    emStatus = GXGetString(this->h_device, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = nullptr;
        return emStatus;
    }

    //printf("<Vendor Name : %s>\n", pszVendorName);
    delete[] pszVendorName;
    pszVendorName = nullptr;

    emStatus = GXGetStringLength(this->h_device, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszModelName = new char[nSize];

    emStatus = GXGetString(this->h_device, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = nullptr;
        return emStatus;
    }

    //printf("<Model Name : %s>\n", pszModelName);
    delete[] pszModelName;
    pszModelName = nullptr;

    emStatus = GXGetStringLength(this->h_device, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszSerialNumber = new char[nSize];

    emStatus = GXGetString(this->h_device, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = nullptr;
        return emStatus;
    }

    //printf("<Serial Number : %s>\n", pszSerialNumber);
    delete[] pszSerialNumber;
    pszSerialNumber = nullptr;

    emStatus = GXGetStringLength(this->h_device, GX_STRING_DEVICE_VERSION, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }
    char *pszDeviceVersion = new char[nSize];

    emStatus = GXGetString(this->h_device, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = nullptr;
        return emStatus;
    }

    //printf("<Device Version : %s>\n", pszDeviceVersion);
    delete[] pszDeviceVersion;
    pszDeviceVersion = nullptr;
    //printf("***********************************************\n");
    return emStatus;
}

nssc::NSSC_STATUS nssc::ingest::Camera::closeCamera()
{
    if(this->is_closed) { return NSSC_STATUS_SUCCESS; }

    this->is_closed = true;

    NSSC_STATUS status;

    this->stream_running = false;
    this->GXDQ_thread.join();

    status = GXCloseDevice(this->h_device);
    this->h_device = nullptr;
    GXCloseLib();

    this->node->printInfo(this->msg_caller, "Camera closed");
    return status;
}

nssc::NSSC_STATUS nssc::ingest::Camera::startAcquisition()
{
    this->stream_running = true;

    for (int i = 0; i < 7; i++)
    {
        framestruct::MonoFrame* frame = framestruct::MonoFrame::makeFrame(this->node->g_config.frame_config.g_type);
        frame->alloc(this->node, i);

        this->buf_empty.enqueue(frame);
        this->n_empty++;
    }

    this->GXDQ_thread = std::thread(&nssc::ingest::Camera::GXDQBufThreadNDI, this);

    return NSSC_STATUS_SUCCESS;
}

void nssc::ingest::Camera::GXDQBufThreadNDI()
{
    framestruct::FrameBuffer rgbBuf;
    cudaSetDeviceFlags(cudaDeviceMapHost);
    cudaHostAlloc((void **)&rgbBuf.hImageBuf, this->g_nPayloadSize * 3, cudaHostAllocMapped);
    cudaHostGetDevicePointer((void **)&rgbBuf.dImageBuf, (void *) rgbBuf.hImageBuf , 0);

    cv::Mat h_rgb(cv::Size(this->node->g_config.frame_config.cam_x_res, this->node->g_config.frame_config.cam_y_res), CV_8UC3, rgbBuf.hImageBuf);
    cv::cuda::GpuMat d_rgb(cv::Size(this->node->g_config.frame_config.cam_x_res, this->node->g_config.frame_config.cam_y_res), CV_8UC3, rgbBuf.dImageBuf);

    PGX_FRAME_BUFFER pFrameBuffer = nullptr;

    while(this->stream_running.load())
    {
        if(this->n_filled.load() < 1 && this->n_empty.load() > 0)
        {
            NSSC_STATUS status;

            framestruct::MonoFrame* frame;
            this->buf_empty.wait_dequeue(frame);
            this->n_empty--;

            this->cb->Await();
            status = GXSendCommand(this->h_device, GX_COMMAND_TRIGGER_SOFTWARE);
            frame->setTimestamp();
            this->last_trigger = std::chrono::high_resolution_clock::now();

            status = GXDQBuf(this->h_device, &pFrameBuffer, 5000);

            status = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, rgbBuf.hImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);

            status = GXQBuf(this->h_device, pFrameBuffer);

            frame->convert(&rgbBuf);

            this->buf_filled.enqueue(frame);
            this->n_filled++;
        }
        else if (__checkFrameAge() && this->stop_age_check.load())
        {
            framestruct::MonoFrame* frame;

            this->buf_filled.wait_dequeue(frame);
            this->n_filled--;

            this->buf_empty.enqueue(frame);
            this->n_empty++;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    cudaFreeHost(rgbBuf.hImageBuf);
}

bool nssc::ingest::Camera::__checkFrameAge()
{
    auto timestamp = std::chrono::high_resolution_clock::now();
    auto timedif = std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamp - this->last_trigger);

    if (timedif > std::chrono::milliseconds(200))
        return true;

    return false;
}

nssc::framestruct::MonoFrame* nssc::ingest::Camera::getFrame()
{
    
    framestruct::MonoFrame* frame;

    this->buf_filled.wait_dequeue(frame);
    this->n_filled--;

    return frame;    
}

nssc::NSSC_STATUS nssc::ingest::Camera::returnBuf(framestruct::MonoFrame* frame)
{
    this->buf_empty.enqueue(frame);
    this->n_empty++;
    return NSSC_STATUS_SUCCESS;
}