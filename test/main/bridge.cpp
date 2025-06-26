// Libraries
#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_log.h"
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
    gpio_set_level((gpio_num_t)GPIO_NUM_25, true);
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

// void netlight_task(void *pvParameters)
// {
//     NetStatus_t state = OFF, prev_state = UNKNOWN;
//     int16_t count = 0;
//     int64_t now = 0;
//     while (1)
//     {
//         portDISABLE_INTERRUPTS();
//         positivePulseWidth = isrPositiveWidth;
//         negativePulseWidth = isrNegativeWidth;
//         lastPulseStartNegativeTime = isrLastPulseStartNegativeTime;
//         portENABLE_INTERRUPTS();
//         now = esp_timer_get_time();
//         if ((now - lastPulseStartNegativeTime) / 1000 > 3300)
//         {
//             positivePulseWidth = 0;
//             negativePulseWidth = 0;
//         }

//         if (positivePulseWidth / 1000 > 0)
//         {
//             // 200ms +- 10%
//             if (negativePulseWidth / 1000 < 220 && negativePulseWidth / 1000 > 180)
//                 state = REGISTERED;
//         }
//         else
//         {
//             if (gpio_get_level((gpio_num_t)PIN_NETLIGHT) == LOW)
//                 state = OFF;
//             else
//                 state = SEARCHING;
//         }

//         if (state != prev_state)
//         {
//             prev_state = state;
//             printf("\t\t positive=%ld | negative=%ld\r\n", positivePulseWidth / 1000, negativePulseWidth / 1000);
//             switch (state)
//             {
//             case OFF:
//                 printf("\t\t Power off / Sleep\r\n");
//                 break;

//             case REGISTERED:
//                 printf("\t\t Data Transmit / Registered\r\n");
//                 break;

//             case SEARCHING:
//                 printf("\t\t Searching Network\r\n");
//                 break;

//             default:
//                 printf("\t\t UNKNOWN STATE\r\n");
//                 break;
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(30));
//         if (count++ > 1000)
//         {
//             count = 0;
//             printf("\t\t positive=%ld | negative=%ld\r\n", positivePulseWidth / 1000, negativePulseWidth / 1000);
//         }
//     }
// }

void setup()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);

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
    io_conf.pin_bit_mask = GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_OUTPUT_PWRKEY) | GPIO_PIN_SEL(GPIO_NUM_25) | GPIO_PIN_SEL(GPIO_NUM_33); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                                                                         // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                                                                             // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
    gpio_set_level(GPIO_NUM_25, false);
    power_on_modem();

    xTaskCreatePinnedToCore(netlight_task, "netlight_task", 2048, NULL, 5, &netlight_task_handle, 0);
    xTaskCreatePinnedToCore(power_task, "netlight_task", 1024, NULL, 5, &power_task_handle, 0);
}

// --------------------------------------------------
// --------------------------------------------------
void loop()
{
    if (Serial.available())
    {
        Serial1.write(Serial.read());
    }

    // computer to R800C
    if (Serial1.available())
    {
        Serial.write(Serial1.read());
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
