#pragma once

#include "common_hardware_interface.h"

class adeptViper : public commonHardwareInterface
{
    public:
        adeptViper();

        hardware_interface::return_type init();

        hardware_interface::return_type write(std::vector<double> *hw_commands_);

        hardware_interface::return_type read(std::vector<double> *hw_commands_);

        hardware_interface::return_type stop();
};