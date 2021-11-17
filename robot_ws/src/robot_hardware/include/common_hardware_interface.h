#pragma once

#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

typedef enum HARDWARE_TYPES
{
    HARDWARE_NSRA2         = 0,
    HARDWARE_ADEPT_VIPER   = 1,
} HARDWARE_TYPE;

class commonHardwareInterface
{
    public:
        static commonHardwareInterface *make_hardware_interface(HARDWARE_TYPE type);

        virtual hardware_interface::return_type init() = 0;
        virtual hardware_interface::return_type write(std::vector<double> *hw_commands_) = 0;
        virtual hardware_interface::return_type read(std::vector<double> *hw_commands_) = 0;
        virtual hardware_interface::return_type stop() = 0;
};