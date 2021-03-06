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

#ifndef ROBOT_HARDWARE_NSRA2_CONTROL_H_
#define ROBOT_HARDWARE_NSRA2_CONTROL_H_

#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "error_handler.h"

namespace nsra2_control
{
    class NSRA2Control
    {
    public:
        NSRA2Control();

        NSRA_STATUS init();
        NSRA_STATUS sendCommands(std::vector<double> &hw_commands_);
        NSRA_STATUS readStatus(std::vector<double> &hw_status_);

    private:
        NSRA_STATUS _findHardware();

        int16_t __calc_steps(double r_pos, double transmission) const;
        int16_t __calc_steps(double r_pos, int transmission) const;

        serial::Serial *hw_serial;

        const std::string hardware_sn = "Teensyduino USB Serial 8347080";
        const unsigned int baud = 912600;
        const double pi = 2*acos(0.0);
        const int multiplier = 65000;
        bool is_connected = false;
    };

} // namespace nsra2_control

#endif //ROBOT_HARDWARE_NSRA2_CONTROL_H_
