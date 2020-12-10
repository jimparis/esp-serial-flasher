#define DT_DRV_COMPAT espressif_esp_flash

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(esp_serial_loader);

#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>

#include "serial_io.h"

static const struct device *uart_dev;
static const struct device *reset_dev;
static const struct device *boot_dev;
static K_TIMER_DEFINE(loader_timer, NULL, NULL);
static K_SEM_DEFINE(uart_got_event, 0, 1);
static enum uart_event_type last_event;

static void uart_cb(const struct device *dev, struct uart_event *evt, void *ctx)
{
        ARG_UNUSED(dev);
        ARG_UNUSED(ctx);
        switch (evt->type) {
        case UART_TX_DONE:
        case UART_TX_ABORTED:
                last_event = evt->type;
                k_sem_give(&uart_got_event);
        default:
                LOG_DBG("unhandled uart event %d", evt->type);
                break;
        }
}

esp_loader_error_t loader_port_serial_init(const loader_serial_config_t *config)
{
        int ret;
        esp_loader_error_t err;

        /* Config is ignored; we use espressif,esp instance 0 from devicetree */
        (void) config;

        uart_dev = device_get_binding(DT_INST_BUS_LABEL(0));

        printk("UART dev is %p\n", uart_dev);
        printk("UART api is %p\n", uart_dev->api);

        /* Make sure speed is at the default value */
        err = loader_port_change_baudrate(
                DT_PROP(DT_BUS(DT_DRV_INST(0)), current_speed));
        if (err != ESP_LOADER_SUCCESS)
                return err;

        /* Set up UART callback */
        ret = uart_callback_set(uart_dev, uart_cb, NULL);
        if (ret < 0) {
                LOG_ERR("uart callback: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }

        /* Initialize control pins */
        reset_dev = device_get_binding(DT_INST_GPIO_LABEL(0, wifi_reset_gpios));
        ret = gpio_pin_configure(reset_dev,
                                 DT_INST_GPIO_PIN(0, wifi_reset_gpios),
                                 DT_INST_GPIO_FLAGS(0, wifi_reset_gpios)
                                 | GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
                LOG_ERR("reset pin: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }

        boot_dev = device_get_binding(DT_INST_GPIO_LABEL(0, wifi_boot_gpios));
        ret = gpio_pin_configure(boot_dev,
                                 DT_INST_GPIO_PIN(0, wifi_boot_gpios),
                                 DT_INST_GPIO_FLAGS(0, wifi_boot_gpios)
                                 | GPIO_OUTPUT_HIGH);
        if (ret < 0) {
                LOG_ERR("boot pin: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }

        return ESP_LOADER_SUCCESS;
}

esp_loader_error_t loader_port_serial_write(const uint8_t *data, uint16_t size,
                                            uint32_t timeout)
{
        int ret;

        ret = uart_tx(uart_dev, data, size, timeout);
        if (ret < 0) {
                LOG_ERR("serial_write: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }
        k_sem_take(&uart_got_event, K_FOREVER);
        switch (last_event) {
        case UART_TX_DONE:
                return ESP_LOADER_SUCCESS;
        case UART_TX_ABORTED:
                return ESP_LOADER_ERROR_TIMEOUT;
        default:
                LOG_ERR("unexpected event %d", last_event);
                return ESP_LOADER_ERROR_FAIL;
        }
}

esp_loader_error_t loader_port_serial_read(uint8_t *data, uint16_t size,
                                           uint32_t timeout)
{
        int ret;

        ret = uart_rx_enable(uart_dev, data, size, timeout);
        if (ret < 0) {
                LOG_ERR("serial_read: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }
        k_sem_take(&uart_got_event, K_FOREVER);
        switch (last_event) {
        default:
                LOG_ERR("unexpected event %d", last_event);
                return ESP_LOADER_ERROR_FAIL;
        }
}

void loader_port_enter_bootloader(void)
{
        /* Set IO0=0 (enter bootloader) and reset */
        gpio_pin_set(boot_dev, DT_INST_GPIO_PIN(0, wifi_boot_gpios), 0);
        loader_port_reset_target();
        k_msleep(50);
        /* Set IO0=1 */
        gpio_pin_set(boot_dev, DT_INST_GPIO_PIN(0, wifi_boot_gpios), 1);
}

void loader_port_reset_target(void)
{
        /* Hold in reset (active low) */
        gpio_pin_set(reset_dev, DT_INST_GPIO_PIN(0, wifi_reset_gpios), 1);
        k_msleep(1);
        /* Release reset */
        gpio_pin_set(reset_dev, DT_INST_GPIO_PIN(0, wifi_reset_gpios), 0);
}

void loader_port_delay_ms(uint32_t ms)
{
        k_msleep(ms);
}

void loader_port_start_timer(uint32_t ms)
{
        k_timer_start(&loader_timer, K_MSEC(ms), K_NO_WAIT);
}

uint32_t loader_port_remaining_time(void)
{
        return k_timer_remaining_get(&loader_timer);
}

void loader_port_debug_print(const char *str)
{
        LOG_DBG("%s", str);
}

esp_loader_error_t loader_port_change_baudrate(uint32_t baudrate)
{
        int ret;

        /* Reconfigure uart */
        const struct uart_config uart_config = {
                .baudrate = baudrate,
                .parity = UART_CFG_PARITY_NONE,
                .stop_bits = UART_CFG_STOP_BITS_1,
                .data_bits = UART_CFG_DATA_BITS_8
        };
        ret = uart_configure(uart_dev, &uart_config);
        if (ret < 0) {
                LOG_ERR("uart_configure: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }
        return ESP_LOADER_SUCCESS;
}
