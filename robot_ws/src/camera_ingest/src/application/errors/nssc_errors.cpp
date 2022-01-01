#include "nssc_errors.h"

NSSC_ERRORS::NSSC_ERRORS(std::shared_ptr<NSSC>& node)
{
    this->node = node;
}

void NSSC_ERRORS::LogErrorInfo(NSSC_STATUS errorStatus)
{
    if(errorStatus == NSSC_STATUS_SUCCESS)
    {
        return;
    } else if(errorStatus < 0 && errorStatus >= -14)
    {
        _GetGXErrorString(errorStatus);
    } else if(errorStatus == NSSC_CAM_STATUS_INVALID_CAMERA_TYPE)
    {
        RCLCPP_ERROR(this->node->get_logger(), "This app only support color cameras!");
    } else if(errorStatus == NSSC_NDI_STATUS_SENDER_INIT_ERROR)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Error while creating NDI sender");
    } else if(errorStatus == NSSC_CAM_STATUS_NOT_CONNECTED)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Error camera not connected");
    } else if(errorStatus == NSSC_CAM_STATUS_NOT_LOADED)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Error camera not loaded");
    } else
    {
        RCLCPP_ERROR(this->node->get_logger(), "Some other error occurred");
    }
}

void NSSC_ERRORS::_GetGXErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Error when calling GXGetLastError");
        return;
    }
    
    error_info = new char[size];
    if (error_info == NULL)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Failed to allocate memory");
        return ;
    }
    
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(this->node->get_logger(), "Error when calling GXGetLastError");
    }
    else
    {
        RCLCPP_ERROR(this->node->get_logger(), "%s", error_info);
    }

    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

NSSC_STATUS NSSC_ERRORS::getStatus()
{
    return this->globalStatus;
}