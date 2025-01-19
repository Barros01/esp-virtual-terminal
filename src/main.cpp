#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/mcp2515_can_interface.hpp"
#include "isobus/hardware_integration/spi_interface_esp.hpp"
#include "driver/spi_master.h"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/can_stack_logger.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client_update_helper.hpp"
#include "isobus/utility/iop_file_interface.hpp"

#include "console_logger.cpp"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "objectPoolObjects.h"

#include <functional>
#include <iostream>
#include <memory>

// It is discouraged to use global variables, but it is done here for simplicity.
static std::shared_ptr<isobus::VirtualTerminalClient> virtualTerminalClient = nullptr;
static std::shared_ptr<isobus::VirtualTerminalClientUpdateHelper> virtualTerminalUpdateHelper = nullptr;

// Can pins
#define CAN1_INT 21 // CAN1 (IMPLEMENT)
#define CAN1_CS  10 // CAN1 (IMPLEMENT)

#define CAN2_INT 47 // CAN2 (TRACTOR)
#define CAN2_CS  9  // CAN2 (TRACTOR)

#define CAN3_CS 40  // CAN3 (AUX)
#define CAN3_INT 42 // CAN3 (AUX)

#define CAN4_RX 8  // CAN4 (AUX)
#define CAN4_TX 18 // CAN4 (AUX)

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"

#define GPIO_OUTPUT_PIN GPIO_NUM_14  // GPIO14
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_PIN)


constexpr uint32_t CAN_BAUD_RATE = 250000; 
constexpr uint32_t SPI_SPEED = 1000000; 

std::shared_ptr<isobus::SPIInterfaceESP> spiTransactionHandler1;
std::shared_ptr<isobus::SPIInterfaceESP> spiTransactionHandler2;
std::shared_ptr<isobus::SPIInterfaceESP> spiTransactionHandler3;

void initialize_spi()
{
    // Initialize the SPI bus for SPI1_HOST
    spi_bus_config_t buscfg = {
        .mosi_io_num = 11, 
        .miso_io_num = 13, 
        .sclk_io_num = 12, 
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("AgIsoStack", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    // Configure SPI for CAN1 (IMPLEMENT)
    spi_device_interface_config_t devcfg1 = {
        .mode = 0,                    
        .clock_speed_hz = SPI_SPEED, 
        .spics_io_num = CAN1_CS,     
        .queue_size = 128,           
    };
    spiTransactionHandler1 = std::make_shared<isobus::SPIInterfaceESP>(&devcfg1, SPI2_HOST);
    
    // Configure SPI for CAN2 (TRACTOR)
    spi_device_interface_config_t devcfg2 = {
        .mode = 0,                    
        .clock_speed_hz = SPI_SPEED, 
        .spics_io_num = CAN2_CS,     
        .queue_size = 128,           
    };
    spiTransactionHandler2 = std::make_shared<isobus::SPIInterfaceESP>(&devcfg2, SPI2_HOST);

    // Configure SPI for CAN3 (AUX)
    spi_device_interface_config_t devcfg3 = {
        .mode = 0,                    
        .clock_speed_hz = SPI_SPEED, 
        .spics_io_num = CAN3_CS,     
        .queue_size = 128,           
    };
    spiTransactionHandler3 = std::make_shared<isobus::SPIInterfaceESP>(&devcfg3, SPI2_HOST);

    // Initialize the SPI devices
    if (!spiTransactionHandler1->init()) {
        ESP_LOGE("AgIsoStack", "Failed to initialize SPI1");
    }
    if (!spiTransactionHandler2->init()) {
        ESP_LOGE("AgIsoStack", "Failed to initialize SPI2");
    }
    if (!spiTransactionHandler3->init()) {
        ESP_LOGE("AgIsoStack", "Failed to initialize SPI3");
    }
}

// This callback will provide us with event driven notifications of softkey presses from the stack
void handle_softkey_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
    if (event.keyNumber == 0)
    {
        // We have the alarm ACK code, so if we have an active alarm, acknowledge it by going back to the main runscreen
        virtualTerminalUpdateHelper->set_active_data_or_alarm_mask(example_WorkingSet, mainRunscreen_DataMask);
    }

    switch (event.keyEvent)
    {
        case isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased:
        {
            switch (event.objectID)
            {
                case alarm_SoftKey:
                {
                    virtualTerminalUpdateHelper->set_active_data_or_alarm_mask(example_WorkingSet, example_AlarmMask);
                }
                break;

                case acknowledgeAlarm_SoftKey:
                {
                    virtualTerminalUpdateHelper->set_active_data_or_alarm_mask(example_WorkingSet, mainRunscreen_DataMask);
                }
                break;

                default:
                    break;
            }
        }
        break;

        default:
            break;
    }
}

// This callback will provide us with event driven notifications of button presses from the stack
void handle_button_event(const isobus::VirtualTerminalClient::VTKeyEvent &event)
{
    switch (event.keyEvent)
    {
        case isobus::VirtualTerminalClient::KeyActivationCode::ButtonUnlatchedOrReleased:
        case isobus::VirtualTerminalClient::KeyActivationCode::ButtonStillHeld:
        {
            switch (event.objectID)
            {
                case Plus_Button:
                {
                    virtualTerminalUpdateHelper->increase_numeric_value(ButtonExampleNumber_VarNum);
                }
                break;

                case Minus_Button:
                {
                    virtualTerminalUpdateHelper->decrease_numeric_value(ButtonExampleNumber_VarNum);
                }
                break;

                default:
                    break;
            }
        }
        break;

        default:
            break;
    }
}

extern "C" const uint8_t object_pool_start[] asm("_binary_object_pool_iop_start");
extern "C" const uint8_t object_pool_end[] asm("_binary_object_pool_iop_end");

extern "C" void app_main()
{
	// initialize_spi();
    
    // Configure GPIO14 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL, // Pin mask
        .mode = GPIO_MODE_OUTPUT,           // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,  // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE      // Disable interrupts
    };

    // Apply the configuration
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_PIN, 1); // Set GPIO14 high

    // Setup for CAN aux 4 (v42 section controller hardware)
    twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN4_RX, (gpio_num_t)CAN4_TX, TWAI_MODE_NORMAL);
    twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    std::shared_ptr<isobus::CANHardwarePlugin> can4Driver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);

    // Setup for MCP2518 can driver (can1, can2, and can3)
    // std::shared_ptr<isobus::CANHardwarePlugin> can1Driver = std::make_shared<isobus::MCP2515CANInterface>(spiTransactionHandler1.get(), CAN1_CS, CAN1_INT, CAN_BAUD_RATE);
    // std::shared_ptr<isobus::CANHardwarePlugin> can22Driver = std::make_shared<isobus::MCP2515CANInterface>(spiTransactionHandler2.get(), CAN2_CS, CAN2_INT, CAN_BAUD_RATE);
    // std::shared_ptr<isobus::CANHardwarePlugin> can3Driver = std::make_shared<isobus::MCP2515CANInterface>(spiTransactionHandler3.get(), CAN3_CS, CAN3_INT, CAN_BAUD_RATE);

    isobus::CANStackLogger::set_can_stack_logger_sink(&logger);
    isobus::CANStackLogger::set_log_level(isobus::CANStackLogger::LoggingLevel::Info); // Change this to Debug to see more information
    // isobus::CANHardwareInterface::set_number_of_can_channels(4);
    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can4Driver);
    // isobus::CANHardwareInterface::assign_can_channel_frame_handler(1, can1Driver);
    // isobus::CANHardwareInterface::assign_can_channel_frame_handler(2, can22Driver);
    // isobus::CANHardwareInterface::assign_can_channel_frame_handler(3, can3Driver);


    // isobus::CANHardwareInterface::set_periodic_update_interval(10); // 10ms update period matches the default FreeRTOS tick rate of 100Hz

    if (!isobus::CANHardwareInterface::start())
    {
        ESP_LOGE("AgIsoStack", "Failed to start CAN hardware interface");
    }

    if (!can4Driver->get_is_valid())
    {
        ESP_LOGE("AgIsoStack", "CAN4 driver is invalid");
    }

    // if (!can1Driver->get_is_valid())
    // {
    //     ESP_LOGE("AgIsoStack", "CAN1 driver is invalid");
    // }

    // if (!can22Driver->get_is_valid())
    // {
    //     ESP_LOGE("AgIsoStack", "CAN2 driver is invalid");
    // }

    // if (!can3Driver->get_is_valid())
    // {
    //     ESP_LOGE("AgIsoStack", "CAN3 driver is invalid");
    // }

    isobus::NAME TestDeviceNAME(0);

    //! Make sure you change these for your device!!!!
    TestDeviceNAME.set_arbitrary_address_capable(true);
    TestDeviceNAME.set_industry_group(1);
    TestDeviceNAME.set_device_class(0);
    TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::SteeringControl));
    TestDeviceNAME.set_identity_number(2);
    TestDeviceNAME.set_ecu_instance(0);
    TestDeviceNAME.set_function_instance(0);
    TestDeviceNAME.set_device_class_instance(0);
    TestDeviceNAME.set_manufacturer_code(1407);

    const std::uint8_t *testPool = object_pool_start;

    const isobus::NAMEFilter filterVirtualTerminal(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
    const std::vector<isobus::NAMEFilter> vtNameFilters = { filterVirtualTerminal };
    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);
    auto TestPartnerVT = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

    virtualTerminalClient = std::make_shared<isobus::VirtualTerminalClient>(TestPartnerVT, TestInternalECU);
    virtualTerminalClient->set_object_pool(0, testPool, (object_pool_end - object_pool_start) - 1, "ais1");
    virtualTerminalClient->get_vt_soft_key_event_dispatcher().add_listener(handle_softkey_event);
    virtualTerminalClient->get_vt_button_event_dispatcher().add_listener(handle_button_event);
    virtualTerminalClient->initialize(true);

    virtualTerminalUpdateHelper = std::make_shared<isobus::VirtualTerminalClientUpdateHelper>(virtualTerminalClient);
    virtualTerminalUpdateHelper->add_tracked_numeric_value(ButtonExampleNumber_VarNum, 214748364); // In the object pool the output number has an offset of -214748364 so we use this to represent 0.
    virtualTerminalUpdateHelper->initialize();

    while (true)
    {
        // CAN stack runs in other threads. Do nothing forever.
        vTaskDelay(10);
    }

    virtualTerminalClient->terminate();
    isobus::CANHardwareInterface::stop();
}
