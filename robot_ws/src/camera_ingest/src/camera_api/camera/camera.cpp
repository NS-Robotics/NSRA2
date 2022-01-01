#include "camera.h"
#include <chrono>

Camera::Camera(std::shared_ptr<NSSC>& node, std::shared_ptr<CyclicBarrier>& cb) : NSSC_ERRORS(node)
{
    this->node = node;
    this->cb = cb;
}

Camera::~Camera()
{
    CloseCamera();
}

NSSC_STATUS Camera::init()
{
    return GXInitLib();
}

NSSC_STATUS Camera::_PrintDeviceInfo()
{
    //RCLCPP_INFO(this->node->get_logger(), "***********************************************");
    //printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    NSSC_STATUS emStatus = NSSC_STATUS_SUCCESS;

    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszVendorName = new char[nSize];

    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        return emStatus;
    }

    //printf("<Vendor Name : %s>\n", pszVendorName);
    delete[] pszVendorName;
    pszVendorName = NULL;

    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszModelName = new char[nSize];

    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        return emStatus;
    }

    //printf("<Model Name : %s>\n", pszModelName);
    delete[] pszModelName;
    pszModelName = NULL;

    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }

    char *pszSerialNumber = new char[nSize];

    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        return emStatus;
    }

    //printf("<Serial Number : %s>\n", pszSerialNumber);
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    emStatus = GXGetStringLength(this->hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    if(emStatus != NSSC_STATUS_SUCCESS)
    {
        return emStatus;
    }
    char *pszDeviceVersion = new char[nSize];

    emStatus = GXGetString(this->hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != NSSC_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        return emStatus;
    }

    //printf("<Device Version : %s>\n", pszDeviceVersion);
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    //printf("***********************************************\n");
    return emStatus;
}

NSSC_STATUS Camera::CloseCamera()
{
    if(this->is_closed) { return NSSC_STATUS_SUCCESS; }

    this->is_closed = true;

    NSSC_STATUS status;

    this->streamON = false;
    this->GXDQThreadNDI.join();
    
    status = GXCloseDevice(this->hDevice);
    this->hDevice = NULL;
    GXCloseLib();

    this->node->printInfo(this->msgCaller, "Camera closed");
    return status;
}

NSSC_STATUS Camera::startAcquisition()
{
    this->streamON = true;

    for (int i = 0; i < 5; i++)
    {
        monoFrame* frame = monoFrame::make_frame(this->node->g_config.frameConfig.g_type);
        frame->alloc(this->node, i);

        this->emptyFrameBuf.enqueue(frame);
        this->numOfEmpty++;
    }

    this->GXDQThreadNDI = std::thread(&Camera::GXDQBufThreadNDI, this);

    return NSSC_STATUS_SUCCESS;
}

void Camera::GXDQBufThreadNDI()
{
    frameBuffer rgbBuf;
    cudaSetDeviceFlags(cudaDeviceMapHost);
    cudaHostAlloc((void **)&rgbBuf.hImageBuf, this->g_nPayloadSize * 3, cudaHostAllocMapped);
    cudaHostGetDevicePointer((void **)&rgbBuf.dImageBuf, (void *) rgbBuf.hImageBuf , 0);

    cv::Mat h_rgb(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC3, rgbBuf.hImageBuf);
    cv::cuda::GpuMat d_rgb(cv::Size(this->node->g_config.frameConfig.cam_x_res, this->node->g_config.frameConfig.cam_y_res), CV_8UC3, rgbBuf.dImageBuf);

    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    while(this->streamON.load())
    {

        if(this->numOfFilled.load() < 1 && this->numOfEmpty.load() > 0)
        {
            NSSC_STATUS status;

            monoFrame* frame;
            this->emptyFrameBuf.wait_dequeue(frame);
            this->numOfEmpty--;

            this->cb->Await();
            status = GXSendCommand(this->hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
            frame->setTimestamp();

            status = GXDQBuf(this->hDevice, &pFrameBuffer, 5000);

            status = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, rgbBuf.hImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                              RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);

            status = GXQBuf(this->hDevice, pFrameBuffer);

            frame->convert(&rgbBuf);

            this->filledFrameBuf.enqueue(frame);
            this->numOfFilled++;
        } else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    cudaFreeHost(rgbBuf.hImageBuf);
}

monoFrame* Camera::getFrame()
{
    
    monoFrame* frame;

    this->filledFrameBuf.wait_dequeue(frame);
    this->numOfFilled--;

    return frame;    
}

NSSC_STATUS Camera::returnBuf(monoFrame* frame)
{
    this->emptyFrameBuf.enqueue(frame);
    this->numOfEmpty++;
    return NSSC_STATUS_SUCCESS;
}