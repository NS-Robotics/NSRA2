/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Noa Sendlhofer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Noa Sendlhofer

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