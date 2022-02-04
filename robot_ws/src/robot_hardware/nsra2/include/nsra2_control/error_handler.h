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

#ifndef ROBOT_HARDWARE_ERROR_HANDLER_H_
#define ROBOT_HARDWARE_ERROR_HANDLER_H_

namespace nsra2_control
{
    typedef int NSRA_STATUS;

    typedef enum NSRA_STATUS_LIST
    {

        NSRA_STATUS_SUCCESS                 =  0,                               // Success
        NSRA_STATUS_ERROR                   = -1,                               // There is an unspecified internal error that is not expected to occur

        // Serial error codes
        NSRA_SERIAL_DEVICE_READY            =  10,                              // Device open and ready
        NSRA_SERIAL_DEVICE_NOT_FOUND        = -10,                              // Device not found
        NSRA_SERIAL_INIT_ERROR              = -11,                              // Error opening device
        NSRA_SERIAL_SEND_ERROR              = -12,                              // Error sending message

    } NSRA_STATUS_LIST;

} // namespace nsra2_control

#endif //ROBOT_HARDWARE_ERROR_HANDLER_H_
