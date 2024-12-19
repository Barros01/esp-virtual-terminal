#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "esp_log.h"


extern "C" void app_main()
{
        twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_42, TWAI_MODE_NORMAL);
        twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
        twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        std::shared_ptr<isobus::CANHardwarePlugin> canDriver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);

        isobus::CANHardwareInterface::set_number_of_can_channels(1);
        isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);
        isobus::CANHardwareInterface::set_periodic_update_interval(1);

        if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid())
        {
                ESP_LOGE("AgIsoStack", "Failed to start hardware interface, the CAN driver might be invalid");
        }

        isobus::NAME TestDeviceNAME(0);

        //! Make sure you change these for your device!!!!
        //! This is an example device that is using a manufacturer code that is currently unused at time of writing
        TestDeviceNAME.set_arbitrary_address_capable(true);
        TestDeviceNAME.set_industry_group(1);
        TestDeviceNAME.set_device_class(0);
        TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
        TestDeviceNAME.set_identity_number(2);
        TestDeviceNAME.set_ecu_instance(0);
        TestDeviceNAME.set_function_instance(0);
        TestDeviceNAME.set_device_class_instance(0);
        TestDeviceNAME.set_manufacturer_code(64);
        auto TestInternalECU = std::make_shared<isobus::InternalControlFunction>(TestDeviceNAME, 0x81, 0);


        while (true)
        {
                // CAN stack runs in other threads. Do nothing forever.
                vTaskDelay(10);
        }

        isobus::CANHardwareInterface::stop();
}