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

#include "nsra2_control.h"

namespace nsra2_control
{

    NSRA2Control::NSRA2Control()
    {

    }

    NSRA_STATUS NSRA2Control::init()
    {
        if (!this->hw_serial->isOpen())
            return _findHardware();
        else
            return NSRA_SERIAL_DEVICE_READY;
    }


    NSRA_STATUS NSRA2Control::_findHardware()
    {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        auto iter = devices_found.begin();

        std::string port;
        NSRA_STATUS status;

        while (iter != devices_found.end())
        {
            serial::PortInfo device = *iter++;

            if (device.description == this->hardware_sn)
            {
                port = device.port;
                status = NSRA_STATUS_SUCCESS;
                break;
            }
            else
            {
                status = NSRA_SERIAL_DEVICE_NOT_FOUND;
            }
        }

        if (status != NSRA_STATUS_SUCCESS)
        { return status; }

        this->hw_serial = new serial::Serial(port, this->baud, serial::Timeout::simpleTimeout(1000));

        if (this->hw_serial->isOpen())
            return NSRA_SERIAL_DEVICE_READY;
        else
            return NSRA_SERIAL_INIT_ERROR;
    }

    NSRA_STATUS NSRA2Control::sendCommands(std::vector<double> hw_commands_)
    {

    }

} // namespace nsra2_control
