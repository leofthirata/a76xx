// Libraries
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "esp_log.h"
#include <string>

#define HIGH 1
#define LOW 0
// --------------------------------------------------
// Variables
#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_25)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_32)
#define PIN_NUM_SWITCH 4
#define PIN_NETLIGHT 37

const char *TAG = "main";

TaskHandle_t netlight_task_handle;
TaskHandle_t power_task_handle;
static BaseType_t xHigherPriorityTaskWoken;

static esp_timer_handle_t lte_timer;

static void lte_timer_callback(void *arg)
{
    xTaskNotify(power_task_handle, 0x01, eSetBits);
}

void timer_set_timer(esp_timer_handle_t timer, uint64_t timeout)
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    esp_timer_start_once(timer, timeout);
}

void power_on_modem()
{
    ESP_LOGI(TAG, "Power on the modem");
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_POWER_ON, true);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_PWRKEY, 0);
}

volatile unsigned long isrPositiveWidth = 0, isrNegativeWidth = 0, isrLastPulseStartNegativeTime = 0, isrLastPulseStartPositiveTime = 0; // Define Interrupt Service Routine (ISR) pulsewidth
unsigned long positivePulseWidth, negativePulseWidth, lastPulseStartNegativeTime, lastPulseStartPositiveTime;

static void IRAM_ATTR a76xx_netlight_isr(void *arg)
{
    static unsigned long pulseStartPositiveTime, pulseStartNegativeTime; // Start time variable
    int64_t now = esp_timer_get_time();

    if (gpio_get_level((gpio_num_t)PIN_NETLIGHT) == HIGH) // If the change was a RISING edge
    {
        pulseStartPositiveTime = now; // Store the start time (in microseconds)
        isrNegativeWidth = now - pulseStartNegativeTime;
    }
    else // If the change was a FALLING edge
    {
        isrPositiveWidth = now - pulseStartPositiveTime; // Calculate the pulsewidth
        pulseStartNegativeTime = now;
        isrLastPulseStartNegativeTime = now;
    }
    xTaskNotifyFromISR(netlight_task_handle, 0x01, eSetBits, &xHigherPriorityTaskWoken);
}

typedef enum
{
    OFF,
    SEARCHING,
    REGISTERED,
    UNKNOWN = 0xff,
} NetStatus_t;

void netlight_task(void *pvParameters)
{
    NetStatus_t state = UNKNOWN, prev_state = UNKNOWN;
    int16_t count = 0;
    int64_t now = 0;
    while (1)
    {
        auto event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        portDISABLE_INTERRUPTS();
        positivePulseWidth = isrPositiveWidth;
        negativePulseWidth = isrNegativeWidth;
        lastPulseStartNegativeTime = isrLastPulseStartNegativeTime;
        portENABLE_INTERRUPTS();
        now = esp_timer_get_time();
        if ((now - lastPulseStartNegativeTime) / 1000 > 300)
        {
            positivePulseWidth = 0;
            negativePulseWidth = 0;
        }

        timer_set_timer(lte_timer, 5000000);
        // 200ms +- 10%
        if (negativePulseWidth / 1000 < 240 && negativePulseWidth / 1000 > 160)
        {
            state = REGISTERED;

            if (state != prev_state)
            {
                prev_state = state;
                printf("\t\t Data Transmit / Registered\r\n");
            }
        }
    }
}

void power_task(void *pvParameters)
{
    while (1)
    {
        auto event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int level = gpio_get_level((gpio_num_t)PIN_NETLIGHT);

        if (level == 0)
            printf("\t\t Modem OFF\r\n");
        else
            printf("\t\t Modem ON\r\n");
    }
}

extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NETLIGHT);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)PIN_NETLIGHT, a76xx_netlight_isr, (void *)PIN_NETLIGHT);

    const esp_timer_create_args_t args = {
        .callback = &lte_timer_callback,
        .name = "lte_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &lte_timer));

    io_conf.intr_type = GPIO_INTR_DISABLE;                                                                                                                // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                                                                                                      // set as output mode
    io_conf.pin_bit_mask = GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_OUTPUT_PWRKEY) | GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_NUM_33); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                                                                         // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                                                                             // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
    gpio_set_level(GPIO_OUTPUT_POWER_ON, false);
    power_on_modem();

    xTaskCreatePinnedToCore(netlight_task, "netlight_task", 2048, NULL, 5, &netlight_task_handle, 0);
    xTaskCreatePinnedToCore(power_task, "power_task", 1024, NULL, 5, &power_task_handle, 0);

    int main_uart = UART_NUM_1;
    uart_config_t config = {};
    config.baud_rate = 115200;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 122;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(main_uart, 127 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(main_uart, &config));
    ESP_ERROR_CHECK(uart_set_pin(main_uart, 1, 3, -1, -1));
    ESP_ERROR_CHECK(uart_set_mode(main_uart, UART_MODE_UART));
    ESP_ERROR_CHECK(uart_set_rx_timeout(main_uart, 3));

    int lte_uart = UART_NUM_2;
    ESP_ERROR_CHECK(uart_driver_install(lte_uart, 127 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(lte_uart, &config));
    ESP_ERROR_CHECK(uart_set_pin(lte_uart, TXD_PIN, RXD_PIN, -1, -1));
    ESP_ERROR_CHECK(uart_set_mode(lte_uart, UART_MODE_UART));
    ESP_ERROR_CHECK(uart_set_rx_timeout(lte_uart, 3));

    while (1) {
        char rx[512];
        size_t len = uart_read_bytes(main_uart, rx, 512, (10 / portTICK_PERIOD_MS));
        if (len > 0) {
            rx[len] = 0;
    
            uart_write_bytes(lte_uart, rx, len);
        }
        len = 0;

        len = uart_read_bytes(lte_uart, rx, 512, (100 / portTICK_PERIOD_MS));
        if (len > 0) {
            rx[len] = 0;
            uart_write_bytes(main_uart, rx, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
