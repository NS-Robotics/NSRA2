#include "camera.h"

NSSC_STATUS Camera::LoadCamera(char device_serial_number[])
{
    this->camSerial = std::string(device_serial_number);
    this->msgCaller = "Camera " + this->camSerial;
    
    uint32_t ui32DeviceNum = 0;

    NSSC_STATUS status;

    status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        if(ui32DeviceNum <= 0)
        {
            return NSSC_CAM_STATUS_NOT_CONNECTED;
        }
        return status;
    }

    GX_OPEN_PARAM stOpenParam;

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_SN;
    stOpenParam.pszContent = device_serial_number;
    
    status = GXOpenDevice(&stOpenParam, &this->hDevice);
    if(status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        RCLCPP_INFO(this->node->get_logger(), "Cameras not connected!");
        return NSSC_CAM_STATUS_NOT_CONNECTED;         
    }

    status = _PrintDeviceInfo();
    _GX_VERIFY_EXIT(status);

    bool g_bColorFilter = false;  
    status = GXIsImplemented(this->hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    _GX_VERIFY_EXIT(status);

    if (!g_bColorFilter)
    {
        CloseCamera();
        return NSSC_CAM_STATUS_INVALID_CAMERA_TYPE;
    }
    else
    {
        status = GXGetEnum(this->hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &this->g_i64ColorFilter);
        _GX_VERIFY_EXIT(status);
    }

    status = GXGetInt(this->hDevice, GX_INT_PAYLOAD_SIZE, &this->g_nPayloadSize);
    _GX_VERIFY(status);

    status = GXSetEnum(this->hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    _GX_VERIFY_EXIT(status);

    int64_t nPixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    status = GXSetEnum(this->hDevice, GX_ENUM_PIXEL_FORMAT, nPixelFormat);
    _GX_VERIFY_EXIT(status);

    status = GXSetFloat(this->hDevice, GX_FLOAT_EXPOSURE_TIME, this->node->g_config.frameConfig.cam_exposure_time);
    _GX_VERIFY_EXIT(status);

    status = GXSetFloat(this->hDevice, GX_FLOAT_GAIN, this->node->g_config.frameConfig.cam_gain);
    _GX_VERIFY_EXIT(status);

    status = GXSetEnum(this->hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    _GX_VERIFY_EXIT(status);
    
    status = GXSetAcqusitionBufferNumber(this->hDevice, this->nBufferNum);
    _GX_VERIFY_EXIT(status);

    status = GXIsImplemented(this->hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &this->bStreamTransferSize);
    _GX_VERIFY_EXIT(status);

    if(this->bStreamTransferSize)
    {
        status = GXSetInt(this->hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        _GX_VERIFY_EXIT(status);
    }

    status = GXIsImplemented(this->hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &this->bStreamTransferNumberUrb);
    _GX_VERIFY_EXIT(status);

    if(this->bStreamTransferNumberUrb)
    {
        status = GXSetInt(this->hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        _GX_VERIFY_EXIT(status);
    }

    status = GXSetEnum(this->hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ONCE);
    _GX_VERIFY_EXIT(status);

    status = GXStreamOn(this->hDevice);
    if(status != NSSC_STATUS_SUCCESS)
    {
        _GX_VERIFY_EXIT(status);
    }

    this->node->printInfo(this->msgCaller, "Camera loaded successfully");
    
    return status;
}