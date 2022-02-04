#pragma once

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "node.h"

#define NODE_VERIFY_EXIT(status)  \
    if (status != NSSC_STATUS_SUCCESS)    \
    {                                     \
        RCLCPP_INFO(node->get_logger(), "Leave spin"); \
        eHandler.LogErrorInfo(status);    \
        rclcpp::shutdown();               \
        return 0;                         \
    }                                                          

typedef int NSSC_STATUS;

typedef enum NSSC_STATUS_LIST
{

    NSSC_STATUS_SUCCESS                 =  0,                               // Success
    NSSC_STATUS_ERROR                   = -1,                               // There is an unspecified internal error that is not expected to occur

    // Error codes -2 up to -14 are reserved for GxIAPI
    NSSC_CAM_STATUS_NOT_FOUND_TL        = GX_STATUS_NOT_FOUND_TL,           // The TL library cannot be found
    NSSC_CAM_STATUS_NOT_FOUND_DEVICE    = GX_STATUS_NOT_FOUND_DEVICE,       // The device is not found
    NSSC_CAM_STATUS_OFFLINE             = GX_STATUS_OFFLINE,                // The current device is in an offline status
    NSSC_CAM_STATUS_INVALID_PARAMETER   = GX_STATUS_INVALID_PARAMETER,      // Invalid parameter. Generally, the pointer is NULL or the input IP and other parameter formats are invalid
    NSSC_CAM_STATUS_INVALID_HANDLE      = GX_STATUS_INVALID_HANDLE,         // Invalid handle
    NSSC_CAM_STATUS_INVALID_CALL        = GX_STATUS_INVALID_CALL,           // The interface is invalid, which refers to software interface logic error
    NSSC_CAM_STATUS_INVALID_ACCESS      = GX_STATUS_INVALID_ACCESS,         // The function is currently inaccessible or the device access mode is incorrect
    NSSC_CAM_STATUS_NEED_MORE_BUFFER    = GX_STATUS_NEED_MORE_BUFFER,       // The user request buffer is insufficient: the user input buffer size during the read operation is less than the actual need
    NSSC_CAM_STATUS_ERROR_TYPE          = GX_STATUS_ERROR_TYPE,             // The type of FeatureID used by the user is incorrect, such as an integer interface using a floating-point function code
    NSSC_CAM_STATUS_OUT_OF_RANGE        = GX_STATUS_OUT_OF_RANGE,           // The value written by the user is crossed
    NSSC_CAM_STATUS_NOT_IMPLEMENTED     = GX_STATUS_NOT_IMPLEMENTED,        // This function is not currently supported
    NSSC_CAM_STATUS_NOT_INIT_API        = GX_STATUS_NOT_INIT_API,           // There is no call to initialize the interface
    NSSC_CAM_STATUS_TIMEOUT             = GX_STATUS_TIMEOUT,                // Timeout error

    // NSSC camera error codes
    NSSC_CAM_STATUS_INIT_ERROR          = -15,                              // Error initalizing camera class
    NSSC_CAM_STATUS_INVALID_CAMERA_TYPE = -16,                              // This app only supports color cameras
    NSSC_CAM_STATUS_NOT_CONNECTED       = -17,                              // Camera not connected
    NSSC_CAM_STATUS_NOT_LOADED          = -18,                              // Camera not loaded

    // NDI error codes
    NSSC_NDI_STATUS_SENDER_INIT_ERROR   = -30,                              // Error creating NDI sender

    //CLI error codes
    NSSC_CLI_ARGUMENT_TYPE_ERROR        = -40,                              // Invalid argument

    //Calibration error codes
    NSSC_CALIB_FILE_NOT_FOUND           = -50,
    NSSC_CALIB_CBC_NOT_FOUND            = -51,

    //Triangulation error codes
    NSSC_TRIANGULATION_FILE_NOT_FOUND   = -60,
    NSSC_TRIANGULATION_MARKERS_NOT_FOUND= -61

} NSSC_STATUS_LIST;

class NSSC_ERRORS
{
    public:
        NSSC_ERRORS(std::shared_ptr<NSSC>& node);

        #define _GX_VERIFY_EXIT(status) \
            if (status != NSSC_STATUS_SUCCESS)        \
            {                                         \
                GXCloseDevice(this->h_device);         \
                this->h_device = NULL;                 \
                GXCloseLib();                         \
                return status;                        \
            }
        
        #define _GX_VERIFY(status) \
            if (status != NSSC_STATUS_SUCCESS)        \
            {                                         \
                return status;                        \
            }
        
        #define _CM_VERIFY_EXIT(status) \
            if (status != NSSC_STATUS_SUCCESS)        \
            {                                         \
                this->cam1->closeCamera();            \
                this->cam2->closeCamera();            \
                return status;                        \
            }

        NSSC_STATUS getStatus();
        void LogErrorInfo(NSSC_STATUS errorStatus);

    protected:
        NSSC_STATUS globalStatus;

    private:
        void _GetGXErrorString(GX_STATUS emErrorStatus);
        std::shared_ptr<NSSC> node;
};