#include "camera.h"
#include "nssc_errors.h"

nssc::NSSC_STATUS nssc::ingest::Camera::setExposure(float exposure_time)
{
    this->node->g_config.frame_config.cam_exposure_time = exposure_time;
    return GXSetFloat(this->h_device, GX_FLOAT_EXPOSURE_TIME, this->node->g_config.frame_config.cam_exposure_time);
}

nssc::NSSC_STATUS nssc::ingest::Camera::setGain(float gain)
{
    this->node->g_config.frame_config.cam_gain = gain;
    return GXSetFloat(this->h_device, GX_FLOAT_GAIN, this->node->g_config.frame_config.cam_gain);
}

nssc::NSSC_STATUS nssc::ingest::Camera::loadCamera(char device_serial_number[])
{
    this->cam_serial = std::string(device_serial_number);
    this->msg_caller = "Camera " + this->cam_serial;
    
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
    
    status = GXOpenDevice(&stOpenParam, &this->h_device);
    if(status != GX_STATUS_SUCCESS)
    {
        GXCloseLib();
        this->node->printError(this->msg_caller, "Cameras not connected!");
        return NSSC_CAM_STATUS_NOT_CONNECTED;
    }

    status = _printDeviceInfo();
    _GX_VERIFY_EXIT(status);

    bool g_bColorFilter = false;  
    status = GXIsImplemented(this->h_device, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    _GX_VERIFY_EXIT(status);

    if (!g_bColorFilter)
    {
        closeCamera();
        return NSSC_CAM_STATUS_INVALID_CAMERA_TYPE;
    }
    else
    {
        status = GXGetEnum(this->h_device, GX_ENUM_PIXEL_COLOR_FILTER, &this->g_i64ColorFilter);
        _GX_VERIFY_EXIT(status);
    }

    status = GXGetInt(this->h_device, GX_INT_PAYLOAD_SIZE, &this->g_nPayloadSize);
    _GX_VERIFY(status);

    status = GXSetEnum(this->h_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    _GX_VERIFY_EXIT(status);

    int64_t nPixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
    status = GXSetEnum(this->h_device, GX_ENUM_PIXEL_FORMAT, nPixelFormat);
    _GX_VERIFY_EXIT(status);

    status = GXSetFloat(this->h_device, GX_FLOAT_EXPOSURE_TIME, this->node->g_config.frame_config.cam_exposure_time);
    _GX_VERIFY_EXIT(status);

    status = GXSetFloat(this->h_device, GX_FLOAT_GAIN, this->node->g_config.frame_config.cam_gain);
    _GX_VERIFY_EXIT(status);

    status = GXSetEnum(this->h_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    _GX_VERIFY_EXIT(status);
    
    status = GXSetAcqusitionBufferNumber(this->h_device, this->nBufferNum);
    _GX_VERIFY_EXIT(status);

    status = GXIsImplemented(this->h_device, GX_DS_INT_STREAM_TRANSFER_SIZE, &this->bStreamTransferSize);
    _GX_VERIFY_EXIT(status);

    if(this->bStreamTransferSize)
    {
        status = GXSetInt(this->h_device, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        _GX_VERIFY_EXIT(status);
    }

    status = GXIsImplemented(this->h_device, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &this->bStreamTransferNumberUrb);
    _GX_VERIFY_EXIT(status);

    if(this->bStreamTransferNumberUrb)
    {
        status = GXSetInt(this->h_device, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        _GX_VERIFY_EXIT(status);
    }

    status = GXSetEnum(this->h_device, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ONCE);
    _GX_VERIFY_EXIT(status);

    status = GXStreamOn(this->h_device);
    if(status != NSSC_STATUS_SUCCESS)
    {
        _GX_VERIFY_EXIT(status);
    }

    this->node->printInfo(this->msg_caller, "Camera loaded successfully");
    
    return status;
}