#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_SETTINGS_LENGTH 16
#define SETTINGS_PREFIX "cpi_cycle"

    /* device data structure */
    struct pixart_data
    {
        const struct device *dev;
        bool sw_smart_flag; // for paw3395 smart algorithm

        struct gpio_callback irq_gpio_cb; // motion pin irq callback
        struct k_work trigger_work;       // realtrigger job

        struct gpio_callback cpi_gpio_cb;     // cpi pin callback
        struct k_work_delayable set_cpi_work; // set cpi job
        uint8_t current_cpi_index;

#if IS_ENABLED(CONFIG_SETTINGS)
        const char settings_key[MAX_SETTINGS_LENGTH];
        struct k_work_delayable save_work;
#endif

        struct k_work_delayable init_work; // the work structure for delayable init steps
        int async_init_step;
        
        uint8_t init_retry_attempts;
        uint8_t init_retry_count;
        uint8_t data_index;

        bool ready; // whether init is finished successfully
        int err;    // error code during async init
    };

    // device config data structure
    struct pixart_config
    {
        struct spi_dt_spec spi;
        struct gpio_dt_spec irq_gpio;
        struct gpio_dt_spec cpi_gpio;
        uint8_t cpi_count;
        bool swap_xy;
        bool inv_x;
        bool inv_y;
        uint8_t evt_type;
        uint8_t x_input_code;
        uint8_t y_input_code;
        bool force_awake;
        uint8_t init_retry_count;
        uint16_t init_retry_interval;
        uint16_t cpi[];
    };

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
