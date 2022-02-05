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

#include "crc/CRC.h"
#include <base64/base64_rfc4648.hpp>

using base64 = cppcodec::base64_rfc4648;

namespace nsra2_control
{

    NSRA2Control::NSRA2Control()
    {
    }

    NSRA_STATUS NSRA2Control::init()
    {
        if (!this->is_connected)
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
                status = NSRA_SERIAL_DEVICE_NOT_FOUND;
        }

        if (status != NSRA_STATUS_SUCCESS)
        {
            return status;
        }

        this->hw_serial = new serial::Serial(port, this->baud, serial::Timeout::simpleTimeout(1000));

        if (this->hw_serial->isOpen())
        {
            this->is_connected = true;
            return NSRA_SERIAL_DEVICE_READY;
        }
        else
            return NSRA_SERIAL_INIT_ERROR;
    }

    /*
     *  Serial buffer:
     *  uint16_t for each axis  --> 12 bytes
     *  gripper command         --> 1 byte
     *  uint32_t crc data       --> 4 bytes
     *
     *  | 12 bytes axis | 1 byte gripper | 4 bytes crc |  --> 17 bytes
     */
    NSRA_STATUS NSRA2Control::sendCommands(std::vector<double> &hw_commands_)
    {
        const int buffer_size = 13;
        unsigned char data[buffer_size];
        std::string out;

        int16_t steps;
        for (size_t i = 0; i < hw_commands_.size() - 2; i++)
        {
            steps = __calc_steps(hw_commands_[i], this->multiplier);

            out.append(std::to_string(steps));
            out.append(" - ");

            data[i*2] = ((uint16_t)(steps + 32000) >> 0) & 0xFF;
            data[i*2+1] = ((uint16_t)(steps + 32000) >> 8) & 0xFF;

            out.append(std::to_string((float)((uint16_t)((data[i*2+1] << 8) | data[i*2]) - 32000) / 65000.0));
            out.append(", ");
        }

        if(hw_commands_[6] > 0.01)
        {
            data[12] = (uint8_t)1;
            out.append("1");
        }
        else
        {
            data[12] = (uint8_t)0;
            out.append("0");
        }

        RCLCPP_INFO(rclcpp::get_logger("NSRA2SystemPositionHardware"), out);

        uint32_t crc = CRC::Calculate(data, buffer_size, CRC::CRC_32());

        unsigned char crc_data[buffer_size + 4];
        for(int n = 0; n < buffer_size; n++) {
            crc_data[n] = data[n];
        }
        crc_data[buffer_size]     = ((uint32_t)crc >> 0) & 0xFF;
        crc_data[buffer_size + 1] = ((uint32_t)crc >> 8) & 0xFF;
        crc_data[buffer_size + 2] = ((uint32_t)crc >> 16) & 0xFF;
        crc_data[buffer_size + 3] = ((uint32_t)crc >> 24) & 0xFF;

        std::string result;
        base64::encode(result, crc_data);
        result.insert(0, 1, '\n');

        unsigned char message[result.length()];
        strcpy((char*)message, result.c_str());

        if (hw_serial->write(message, result.length()) != result.length())
        {
            RCLCPP_ERROR(
                    rclcpp::get_logger("NSRA2SystemPositionHardware"), "Error writing to serial port");
            return NSRA_SERIAL_SEND_ERROR;
        }
        else
            return NSRA_STATUS_SUCCESS;
    }

    int16_t NSRA2Control::__calc_steps(double r_pos, double transmission) const
    {
        return round(r_pos / pi * 0.5 * transmission);
    }

    int16_t NSRA2Control::__calc_steps(double r_pos, int transmission) const
    {
        return round(r_pos / pi * 0.5 * transmission);
    }

    NSRA_STATUS NSRA2Control::readStatus(std::vector<double> &hw_status_)
    {
        return 0;
    }

} // namespace nsra2_control
