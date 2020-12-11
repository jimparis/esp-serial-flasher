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
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
static const struct device *power_dev;
#endif
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
static const struct device *reset_dev;
#endif
static const struct device *boot_dev;
static K_TIMER_DEFINE(loader_timer, NULL, NULL);
K_MSGQ_DEFINE(rx_queue, 1, 64, 1);
K_MSGQ_DEFINE(tx_queue, 1, 64, 1);
static K_SEM_DEFINE(tx_queue_empty, 0, 1);

static void uart_isr_cb(const struct device *dev, void *ctx)
{
        ARG_UNUSED(ctx);
        uint8_t ch;

        while (uart_irq_update(dev) && uart_irq_is_pending(dev))
        {
                /* Anything to transmit? */
                if (uart_irq_tx_ready(dev))
                {
                        if (k_msgq_get(&tx_queue, &ch, K_NO_WAIT) >= 0) {
                                uart_fifo_fill(dev, &ch, 1);
                        } else {
                                k_sem_give(&tx_queue_empty);
                                uart_irq_tx_disable(dev);
                        }
                }

                /* Anything to receive? */
                if (uart_irq_rx_ready(dev))
                {
                        while (uart_fifo_read(dev, &ch, sizeof(ch)))
                                k_msgq_put(&rx_queue, &ch, K_NO_WAIT);
                }
        }
}

esp_loader_error_t loader_port_serial_init(const loader_serial_config_t *config)
{
        int ret;
        esp_loader_error_t err;

        /* Config is ignored; we use espressif,esp instance 0 from devicetree */
        (void) config;

        uart_dev = device_get_binding(DT_INST_BUS_LABEL(0));

        /* Make sure speed is at the default value */
        err = loader_port_change_baudrate(
                DT_PROP(DT_BUS(DT_DRV_INST(0)), current_speed));
        if (err != ESP_LOADER_SUCCESS)
                return err;

        /* Initialize control pins */
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        power_dev = device_get_binding(DT_INST_GPIO_LABEL(0, power_gpios));
        ret = gpio_pin_configure(power_dev,
                                 DT_INST_GPIO_PIN(0, power_gpios),
                                 DT_INST_GPIO_FLAGS(0, power_gpios)
                                 | GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
                LOG_ERR("power pin: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }
#endif
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
        reset_dev = device_get_binding(DT_INST_GPIO_LABEL(0, reset_gpios));
        ret = gpio_pin_configure(reset_dev,
                                 DT_INST_GPIO_PIN(0, reset_gpios),
                                 DT_INST_GPIO_FLAGS(0, reset_gpios)
                                 | GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
                LOG_ERR("reset pin: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }
#endif

        boot_dev = device_get_binding(DT_INST_GPIO_LABEL(0, boot_gpios));
        ret = gpio_pin_configure(boot_dev,
                                 DT_INST_GPIO_PIN(0, boot_gpios),
                                 DT_INST_GPIO_FLAGS(0, boot_gpios)
                                 | GPIO_OUTPUT_HIGH);
        if (ret < 0) {
                LOG_ERR("boot pin: %d", ret);
                return ESP_LOADER_ERROR_FAIL;
        }

        /* Enable rx interrupts */
        uart_irq_callback_set(uart_dev, uart_isr_cb);
        uart_irq_rx_enable(uart_dev);

        return ESP_LOADER_SUCCESS;
}

esp_loader_error_t loader_port_serial_write(const uint8_t *data, uint16_t size,
                                            uint32_t timeout)
{
        uint64_t end = z_timeout_end_calc(K_MSEC(timeout));
        int64_t remaining;

        while (size) {
                remaining = end - z_tick_get();
                if (remaining <= 0)
                        goto timeout;
                if (k_msgq_put(&tx_queue, data, Z_TIMEOUT_TICKS(remaining)) < 0)
                        goto timeout;
                uart_irq_tx_enable(uart_dev);
                data++;
                size--;
        }
        remaining = end - z_tick_get();
        if (k_sem_take(&tx_queue_empty, Z_TIMEOUT_TICKS(remaining)) < 0)
                goto timeout;
        return ESP_LOADER_SUCCESS;
timeout:
        return ESP_LOADER_ERROR_TIMEOUT;
}

esp_loader_error_t loader_port_serial_read(uint8_t *data, uint16_t size,
                                           uint32_t timeout)
{
        uint64_t end = z_timeout_end_calc(K_MSEC(timeout));
        int64_t remaining;

        while (size) {
                remaining = end - z_tick_get();
                if (remaining <= 0)
                        goto timeout;
                if (k_msgq_get(&rx_queue, data, Z_TIMEOUT_TICKS(remaining)) < 0)
                        goto timeout;
                data++;
                size--;
        }
        return ESP_LOADER_SUCCESS;
timeout:
        return ESP_LOADER_ERROR_TIMEOUT;
}

void loader_port_enter_bootloader(void)
{
        /* Set IO0=0 (enter bootloader) and reset */
        gpio_pin_set(boot_dev, DT_INST_GPIO_PIN(0, boot_gpios), 0);
        loader_port_reset_target();
        k_msleep(50);
        /* Set IO0=1 */
        gpio_pin_set(boot_dev, DT_INST_GPIO_PIN(0, boot_gpios), 1);
}

void loader_port_reset_target(void)
{
        /* Turn power off, or hold in reset */
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        gpio_pin_set(power_dev, DT_INST_GPIO_PIN(0, power_gpios), 0);
        k_msleep(1);
        gpio_pin_set(power_dev, DT_INST_GPIO_PIN(0, power_gpios), 1);
#elif DT_INST_NODE_HAS_PROP(0, reset_gpios)
        gpio_pin_set(reset_dev, DT_INST_GPIO_PIN(0, reset_gpios), 1);
        k_msleep(1);
        gpio_pin_set(reset_dev, DT_INST_GPIO_PIN(0, reset_gpios), 0);
#else
        #error Need to define power_gpios or reset_gpios
#endif
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
