#include "common_hardware_interface.h"
#include "nsra2/nsra2_hardware.h"
#include "adept_viper/adept_viper_hardware.h"

commonHardwareInterface *commonHardwareInterface::make_hardware_interface(HARDWARE_TYPE type)
{
    switch(type)
    {
        case HARDWARE_NSRA2:
            return new NSRA2;
            break;
        case HARDWARE_ADEPT_VIPER:
            return new adeptViper;
            break;
    }
}
