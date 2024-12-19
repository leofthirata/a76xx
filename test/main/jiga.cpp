#include "sdkconfig.h"

#include "Arduino.h"

#include "A76XX/a76xx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"

#include "LedBicolor.hpp"

#define A7683_TXD_PIN         14
#define A7683_RXD_PIN         34
#define A7683_PWRKEY          16
#define A7683_POWER_ON        13
#define A7683_NETLIGHT        37
#define PIN_LED_STATUS_RED  GPIO_NUM_33
#define PIN_LED_STATUS_GREEN GPIO_NUM_25

#define STORAGE_NAMESPACE "lbs"

TaskHandle_t rx_task_handle;
TaskHandle_t tx_task_handle;
static QueueHandle_t xQueueModemRx;

typedef struct
{
    int len;
    char rx[512];
} ModemUartRx_t;

bool lte_timer_up = false;
static esp_timer_handle_t lte_timer;
static LedBicolor *rg;

typedef enum
{
    LTE_STATE_ATE,
    LTE_STATE_CREG,
    LTE_STATE_CGREG,
    LTE_STATE_CMEE,
    LTE_STATE_NETCLOSE,
    LTE_STATE_CGDCONT_1,
    LTE_STATE_CGDCONT_2,
    LTE_STATE_CSOCKSETPN,
    LTE_STATE_CIPMODE,
    LTE_STATE_NETOPEN,
    LTE_STATE_IPADDR,
    LTE_STATE_CIPOPEN,
    LTE_STATE_CIPSEND,
    LTE_STATE_CIPCLOSE,
    LTE_STATE_MSG,
    LTE_STATE_WAITING,
    LTE_STATE_ERROR,
    LTE_STATE_DONE,
    LTE_STATE_POWER_OFF,
} LTEState_t;

static void lte_timer_callback(void *arg)
{
    lte_timer_up = true;
}

void timer_set_timer(esp_timer_handle_t timer, uint64_t timeout)
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    esp_timer_start_once(timer, timeout);
}

void uart_config()
{
    const int uart_num = UART_NUM_1;
    uart_config_t config = {};
    config.baud_rate = 115200;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 122;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(uart_num, 127 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, A7683_TXD_PIN, A7683_RXD_PIN, -1, -1));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_UART));
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, 3));
}

void rx_task(void *args) // UART Receive Task
{
    static const char *tag = "SIM_RX_TASK";
    ModemUartRx_t response;

    uart_config();

    ESP_LOGW(tag, "RX TASK BEGIN\r\n");

    while (1)
    {
        response.len = uart_read_bytes(UART_NUM_1, response.rx, 512, (100 / portTICK_PERIOD_MS));
        if (response.len > 0)
        {
            response.rx[response.len] = 0;

            // 5.2.15 AT+CGEREP GPRS event reporting
            if (strstr(response.rx, "+CPIN: READY") != NULL)
                m_simcard = true;
                // callbacks
            if (strstr(response.rx, "+CPIN: SIM REMOVED") != NULL)
                m_simcard = false;
            // The network has activated a context.
            if (strstr(response.rx, "+CGEV: NW PDN ACT") != NULL)
                m_status = REGISTERED;
            if (strstr(response.rx, "+CGEV: NW PDN DEACT") != NULL)
                m_status = SEARCHING;

            xQueueSend(xQueueModemRx, &response, NULL);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// jiga
void test_lte()
{
    printf("TEST LTE BEGIN\r\n");

    const esp_timer_create_args_t args = {
        .callback = &lte_timer_callback,
        .name = "lte_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &lte_timer));

    LTEState_t lte_test_state_write = LTE_STATE_ATE, lte_test_state_read = LTE_STATE_ATE;
    char cmd[128] = {};
    bool lte_test_done = false;
    String rx;
    uint8_t error_count = 0;

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    if(esp_timer_is_active(lte_timer))
        esp_timer_stop(lte_timer);

    lte_timer_up = false;

    while (!lte_test_done && !lte_timer_up)
    {
        switch(lte_test_state_write)
        {
            case LTE_STATE_ATE:
            {
                //printf("lte_test_state_write LTE_STATE_ATE\r");
                Serial1.write("ATE1\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CREG:
            {
                //printf("lte_test_state_write LTE_STATE_CLCK\r");
                Serial1.write("AT+CREG?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGREG:
            {
                //printf("lte_test_state_write LTE_STATE_CIPRXGET\r");
                Serial1.write("AT+CGREG?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CMEE:
            {
                //printf("lte_test_state_write LTE_STATE_CSTT\r");
                Serial1.write("AT+CMEE=1\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            // TIMER
            case LTE_STATE_NETCLOSE:
            {
                //printf("lte_test_state_write LTE_STATE_CIICR\r");
                Serial1.write("AT+NETCLOSE\r");
                timer_set_timer(lte_timer, 120000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGDCONT_1:
            {
                //printf("lte_test_state_write LTE_STATE_CIFSR\r");
                Serial1.write("AT+CGDCONT=1,\"IP\",\"simplepm.algar.br\",\"0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0\",0,0\r");
                timer_set_timer(lte_timer, 9000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGDCONT_2:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSTART\r");
                Serial1.write("AT+CGDCONT?\r");
                timer_set_timer(lte_timer, 75000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CSOCKSETPN:
            {
                //printf("lte_test_state_write LTE_STATE_CIPRXGET2\r");
                Serial1.write("AT+CSOCKSETPN=1\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPMODE:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSEND\r");
                Serial1.write("AT+CIPMODE=0\r");
                timer_set_timer(lte_timer, 30000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_NETOPEN:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSHUT\r");
                Serial1.write("AT+NETOPEN\r");
                timer_set_timer(lte_timer, 65000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_IPADDR:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+IPADDR\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPOPEN:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPOPEN=0,\"TCP\",\"0.tcp.sa.ngrok.io\",19017\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPSEND:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPSEND=0,5\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPCLOSE:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPCLOSE=0\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_MSG:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("TEST\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_WAITING:
            {
                break;
            }
            case LTE_STATE_ERROR:
            {
                lte_test_state_write = LTE_STATE_CIPCLOSE;
                lte_test_state_read = LTE_STATE_CIPCLOSE;

                error_count++;
                if (error_count == 3)
                {
                    lte_timer_up = false;
                    error_count = 0;
                    printf("JIGA ISCA GSM FAIL. TRIED 3 TIMES\r\n");
                    lte_test_state_write = LTE_STATE_POWER_OFF;
                    lte_test_state_read = LTE_STATE_POWER_OFF;
                }
                break;
            }
            case LTE_STATE_DONE:
            {
                break;
            }
            case LTE_STATE_POWER_OFF:
            {
                //printf("lte_test_state_write LTE_STATE_POWER_OFF\r");
                Serial1.write("AT+CPOF\r");
                timer_set_timer(lte_timer, 60000000); // sets timer for 15s for gsm testing
                break;
            }
        } // switch write end

        if (Serial1.available())
        {
            // Serial.write(Serial1.read());
            rx = Serial1.readString();
            Serial.println(rx);
            
            switch(lte_test_state_read) // só entra nesse switch se tiver dados na serial
            {
                case LTE_STATE_ATE:
                {
                    // printf("lte_test_state_read LTE_STATE_ATE\r");
                    // printf(rx.indexOf("OK"));

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CREG;
                        lte_test_state_read = LTE_STATE_CREG;
                    }

                    break;
                }
                case LTE_STATE_CREG:
                {
                    //printf("lte_test_state_read LTE_STATE_CREG\r");

                    if (rx.indexOf("+CREG: 0,1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGREG;
                        lte_test_state_read = LTE_STATE_CGREG;
                    }
/* 0 not registered, ME is not currently searching a new operator to
register to.
1 registered, home network.
2 not registered, but ME is currently searching a new operator to
register to.
3 registration denied.
4 unknown.
5 registered, roaming.
6 registered for "SMS only", home network (applicable only when
E-UTRAN)
*/
                    else 
                    {
                        // printf("NOT REGISTERED\r\n");
                        lte_test_state_write = LTE_STATE_CGREG;
                        lte_test_state_read = LTE_STATE_CGREG;
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                    }
                    
                    break;
                }
                case LTE_STATE_CGREG:
                {
                    //printf("lte_test_state_read LTE_STATE_CGREG\r");

                    if (rx.indexOf("+CGREG: 0,1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CMEE;
                        lte_test_state_read = LTE_STATE_CMEE;
                    }
/*
0 not registered, ME is not currently searching an operator to register
to
1 registered, home network
2 not registered, but ME is currently trying to attach or searching an
operator to register to
3 registration denied
4 unknown
5 registered, roaming
6 registered for "SMS only", home network(applicable only when
E-UTRAN)
11 attached for emergency bearer services only
*/
                    else
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CMEE:
                {
                    //printf("lte_test_state_read LTE_STATE_CMEE\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_NETCLOSE;
                        lte_test_state_read = LTE_STATE_NETCLOSE;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_NETCLOSE:
                {
                    //printf("lte_test_state_read LTE_STATE_NETCLOSE\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_1;
                        lte_test_state_read = LTE_STATE_CGDCONT_1;
                    }
                    else if (rx.indexOf("+NETCLOSE: 2") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_1;
                        lte_test_state_read = LTE_STATE_CGDCONT_1;
                    }
                    break;
                }
                case LTE_STATE_CGDCONT_1:
                {
                    //printf("lte_test_state_read LTE_STATE_CGDCONT_1\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_2;
                        lte_test_state_read = LTE_STATE_CGDCONT_2;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }                    
                    break;
                }
                case LTE_STATE_CGDCONT_2:
                {
                    //printf("lte_test_state_read LTE_STATE_CGDCONT_2\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CSOCKSETPN;
                        lte_test_state_read = LTE_STATE_CSOCKSETPN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CSOCKSETPN:
                {
                    //printf("lte_test_state_read LTE_STATE_CSOCKSETPN\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPMODE;
                        lte_test_state_read = LTE_STATE_CIPMODE;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPMODE:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPMODE\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_NETOPEN;
                        lte_test_state_read = LTE_STATE_NETOPEN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_NETOPEN:
                {
                    //printf("lte_test_state_read LTE_STATE_NETOPEN\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_IPADDR;
                        lte_test_state_read = LTE_STATE_IPADDR;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_IPADDR:
                {
                    //printf("lte_test_state_read LTE_STATE_IPADDR\r");

                    if (rx.indexOf("+IPADDR:") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPOPEN;
                        lte_test_state_read = LTE_STATE_CIPOPEN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPOPEN:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPOPEN\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPSEND;
                        lte_test_state_read = LTE_STATE_CIPSEND;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPSEND:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPSEND\r");

                    // if (rx.indexOf("> test") != -1)
                    lte_test_state_write = LTE_STATE_MSG;
                    lte_test_state_read = LTE_STATE_MSG;
                    
                    break;
                }
                case LTE_STATE_MSG:
                {
                    //printf("lte_test_state_write LTE_STATE_MSG\r");

                    if (rx.indexOf("TEST") != -1)
                    {
                        lte_test_state_write = LTE_STATE_DONE;
                        lte_test_state_read = LTE_STATE_DONE;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPCLOSE:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPCLOSE\r");

                    if (rx.indexOf("+CIPRXGET: 1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ATE;
                        lte_test_state_read = LTE_STATE_ATE;
                    }
                    else if (rx.indexOf("CONNECT FAIL") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_WAITING:
                {
                    break;
                }
                case LTE_STATE_ERROR:
                {
                    // error_count++;

                    // lte_test_state_write = LTE_STATE_CIPSHUT;
                    // lte_test_state_read = LTE_STATE_CIPSHUT;

                    // if (error_count == 4)
                    // {
                    //     error_count = 0;
                    //     printf("JIGA ISCA GSM FAIL. TRIED 3 TIMES");
                    //     lte_test_done = true;
                    //     xTaskNotify(m_otp_task, 0, eSetValueWithOverwrite);
                    // }
                    break;
                }
                case LTE_STATE_DONE:
                {
                    printf("JIGA ISCA GSM OK\r\n");
                    lte_test_state_write = LTE_STATE_POWER_OFF;
                    lte_test_state_read = LTE_STATE_POWER_OFF;
                    break;
                }
                case LTE_STATE_POWER_OFF:
                {
                    if (rx.indexOf("NORMAL POWER DOWN") != -1)
                        printf("POWER DOWN SUCCESS\r\n");

                    // power_off_modem();
                    lte_test_done = true;
                    esp_timer_stop(lte_timer);
                    esp_timer_delete(lte_timer);
                }
            } // switch read end
        }

        // if (Serial1.available())
        // {
        //     // Serial.write(Serial1.read());
        //     rx = Serial1.readString();
        //     // rx.trim();
        //     printf(rx);
        // }

        vTaskDelay(10);
    } // while end

    if (lte_timer_up)
    {
        printf("JIGA ISCA GSM TIMEOUT\r\n");
    }
}

esp_err_t print_what_saved(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    uint32_t id = 0; // value will default to 0, if not set yet in NVS
    err = nvs_get_u32(my_handle, "id", &id);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;
    printf("id = %" PRIu32 "\n", id);

    char key[9];

    size_t len = 40;

    char buf[len];

    if (id > 0)
    {
        for (uint32_t i = 1; i < id; i++) 
        {
            sprintf(key, "%08lx", i);
    
            nvs_get_str(my_handle, key, buf, &len);
            printf("%" PRIu32 ": %s\r\n", i + 1, buf);

        }
    }
    else
    {
        printf("Nothing to be printed yet\r\n");
    }

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


esp_err_t save_latlong(const char *pos)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    uint32_t temp = 0;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    err = nvs_get_u32(my_handle, "id", &temp);

    temp++;

    nvs_erase_key(my_handle, "id");

    err = nvs_set_u32(my_handle, "id", temp);

    nvs_commit(my_handle);

    char key[9];
    sprintf(key, "%08lx", temp);

    char blob[40];
    err = nvs_get_str(my_handle, key, blob, NULL);
    if (err == ESP_ERR_NVS_NOT_FOUND) // ESP_ERR_NVS_NOT_FOUND error occurs if key doesn't exist yet
        err = nvs_set_str(my_handle, key, pos);

    if (err != ESP_OK) return err;

    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

// lbs validation
void test_lbs()
{
    printf("TEST LBS BEGIN\r\n");

    const esp_timer_create_args_t args = {
        .callback = &lte_timer_callback,
        .name = "lte_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &lte_timer));

    LTEState_t lte_test_state_write = LTE_STATE_ATE, lte_test_state_read = LTE_STATE_ATE;
    char cmd[128] = {};
    bool lte_test_done = false;
    String rx;
    uint8_t error_count = 0;

    vTaskDelay(10000 / portTICK_PERIOD_MS);

    if(esp_timer_is_active(lte_timer))
        esp_timer_stop(lte_timer);

    lte_timer_up = false;

    while (!lte_test_done && !lte_timer_up)
    {
        switch(lte_test_state_write)
        {
            case LTE_STATE_ATE:
            {
                //printf("lte_test_state_write LTE_STATE_ATE\r");
                Serial1.write("ATE1\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CREG:
            {
                //printf("lte_test_state_write LTE_STATE_CLCK\r");
                Serial1.write("AT+CREG?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGREG:
            {
                //printf("lte_test_state_write LTE_STATE_CIPRXGET\r");
                Serial1.write("AT+CGREG?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CMEE:
            {
                //printf("lte_test_state_write LTE_STATE_CSTT\r");
                Serial1.write("AT+CPSI?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            // TIMER
            case LTE_STATE_NETCLOSE:
            {
                //printf("lte_test_state_write LTE_STATE_CIICR\r");
                Serial1.write("AT+NETCLOSE\r");
                timer_set_timer(lte_timer, 120000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGDCONT_1:
            {
                //printf("lte_test_state_write LTE_STATE_CIFSR\r");
                Serial1.write("AT+CGDCONT=1,\"IP\",\"simplepm.algar.br\",\"0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0\",0,0\r");
                timer_set_timer(lte_timer, 9000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CGDCONT_2:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSTART\r");
                Serial1.write("AT+CGACT=1,1\r");
                timer_set_timer(lte_timer, 75000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CSOCKSETPN:
            {
                //printf("lte_test_state_write LTE_STATE_CIPRXGET2\r");
                Serial1.write("AT+CGACT?\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPMODE:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSEND\r");
                Serial1.write("AT+SIMEI?\r");
                timer_set_timer(lte_timer, 30000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_NETOPEN:
            {
                //printf("lte_test_state_write LTE_STATE_CIPSHUT\r");
                Serial1.write("AT+CLBS=1\r");
                timer_set_timer(lte_timer, 65000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_IPADDR:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+IPADDR\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPOPEN:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPOPEN=0,\"TCP\",\"0.tcp.sa.ngrok.io\",19017\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPSEND:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPSEND=0,5\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_CIPCLOSE:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("AT+CIPCLOSE=0\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_MSG:
            {
                //printf("lte_test_state_write LTE_STATE_MSG\r");
                Serial1.write("TEST\r");
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_WAITING;
                break;
            }
            case LTE_STATE_WAITING:
            {
                break;
            }
            case LTE_STATE_ERROR:
            {
                lte_test_state_write = LTE_STATE_CIPCLOSE;
                lte_test_state_read = LTE_STATE_CIPCLOSE;

                error_count++;
                if (error_count == 3)
                {
                    lte_timer_up = false;
                    error_count = 0;
                    printf("JIGA ISCA GSM FAIL. TRIED 3 TIMES\r\n");
                    lte_test_state_write = LTE_STATE_POWER_OFF;
                    lte_test_state_read = LTE_STATE_POWER_OFF;
                }
                break;
            }
            case LTE_STATE_DONE:
            {
                break;
            }
            case LTE_STATE_POWER_OFF:
            {
                //printf("lte_test_state_write LTE_STATE_POWER_OFF\r");
                Serial1.write("AT+CPOF\r");
                timer_set_timer(lte_timer, 60000000); // sets timer for 15s for gsm testing
                break;
            }
        } // switch write end

        if (Serial1.available())
        {
            // Serial.write(Serial1.read());
            rx = Serial1.readString();
            Serial.println(rx);
            
            switch(lte_test_state_read) // só entra nesse switch se tiver dados na serial
            {
                case LTE_STATE_ATE:
                {
                    // printf("lte_test_state_read LTE_STATE_ATE\r");
                    // printf(rx.indexOf("OK"));

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CREG;
                        lte_test_state_read = LTE_STATE_CREG;
                    }

                    break;
                }
                case LTE_STATE_CREG:
                {
                    //printf("lte_test_state_read LTE_STATE_CREG\r");

                    if (rx.indexOf("+CREG: 0,1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGREG;
                        lte_test_state_read = LTE_STATE_CGREG;
                    }
/* 0 not registered, ME is not currently searching a new operator to
register to.
1 registered, home network.
2 not registered, but ME is currently searching a new operator to
register to.
3 registration denied.
4 unknown.
5 registered, roaming.
6 registered for "SMS only", home network (applicable only when
E-UTRAN)
*/
                    else 
                    {
                        // printf("NOT REGISTERED\r\n");
                        lte_test_state_write = LTE_STATE_CREG;
                        lte_test_state_read = LTE_STATE_CREG;
                        vTaskDelay(3000 / portTICK_PERIOD_MS);
                    }
                    
                    break;
                }
                case LTE_STATE_CGREG:
                {
                    //printf("lte_test_state_read LTE_STATE_CGREG\r");

                    if (rx.indexOf("+CGREG: 0,1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CMEE;
                        lte_test_state_read = LTE_STATE_CMEE;
                    }
                    else
                    {
                        lte_test_state_write = LTE_STATE_CGREG;
                        lte_test_state_read = LTE_STATE_CGREG;
                    }
                    break;
                }
                case LTE_STATE_CMEE: // +CPSI: LTE,Online,724-04,0x8632,70257162,7,EUTRAN-BAND3,1275,3,0,9,41,44,-4
                {
                    //printf("lte_test_state_read LTE_STATE_CMEE\r");

                    if (rx.indexOf("+CPSI: LTE,Online") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_1;
                        lte_test_state_read = LTE_STATE_CGDCONT_1;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_NETCLOSE:
                {
                    //printf("lte_test_state_read LTE_STATE_NETCLOSE\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_1;
                        lte_test_state_read = LTE_STATE_CGDCONT_1;
                    }
                    else if (rx.indexOf("+NETCLOSE: 2") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_1;
                        lte_test_state_read = LTE_STATE_CGDCONT_1;
                    }
                    break;
                }
                case LTE_STATE_CGDCONT_1:
                {
                    //printf("lte_test_state_read LTE_STATE_CGDCONT_1\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CGDCONT_2;
                        lte_test_state_read = LTE_STATE_CGDCONT_2;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }                    
                    break;
                }
                case LTE_STATE_CGDCONT_2:
                {
                    //printf("lte_test_state_read LTE_STATE_CGDCONT_2\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CSOCKSETPN;
                        lte_test_state_read = LTE_STATE_CSOCKSETPN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CSOCKSETPN:
                {
                    //printf("lte_test_state_read LTE_STATE_CSOCKSETPN\r");

                    if (rx.indexOf("+CGACT: 1,1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPMODE;
                        lte_test_state_read = LTE_STATE_CIPMODE;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPMODE:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPMODE\r");

                    if (rx.indexOf("+SIMEI:") != -1)
                    {
                        lte_test_state_write = LTE_STATE_NETOPEN;
                        lte_test_state_read = LTE_STATE_NETOPEN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_NETOPEN:
                {
                    //printf("lte_test_state_read LTE_STATE_NETOPEN\r");

                    if (rx.indexOf("+CLBS: 0") != -1)
                    {
                        int idx = rx.indexOf(",");
                        int idx2 = rx.lastIndexOf(",");
                        String token = rx.substring(idx+1, idx2);
                        save_latlong(token.c_str());
                        rg->blink(0xFF00, 1000);
                    }
                    lte_test_state_write = LTE_STATE_NETOPEN;
                    lte_test_state_read = LTE_STATE_NETOPEN;

                    vTaskDelay(10000 / portTICK_PERIOD_MS);

                    break;
                }
                case LTE_STATE_IPADDR:
                {
                    //printf("lte_test_state_read LTE_STATE_IPADDR\r");

                    if (rx.indexOf("+IPADDR:") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPOPEN;
                        lte_test_state_read = LTE_STATE_CIPOPEN;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPOPEN:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPOPEN\r");

                    if (rx.indexOf("OK") != -1)
                    {
                        lte_test_state_write = LTE_STATE_CIPSEND;
                        lte_test_state_read = LTE_STATE_CIPSEND;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPSEND:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPSEND\r");

                    // if (rx.indexOf("> test") != -1)
                    lte_test_state_write = LTE_STATE_MSG;
                    lte_test_state_read = LTE_STATE_MSG;
                    
                    break;
                }
                case LTE_STATE_MSG:
                {
                    //printf("lte_test_state_write LTE_STATE_MSG\r");

                    if (rx.indexOf("TEST") != -1)
                    {
                        lte_test_state_write = LTE_STATE_DONE;
                        lte_test_state_read = LTE_STATE_DONE;
                    }
                    else if (rx.indexOf("ERROR") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_CIPCLOSE:
                {
                    //printf("lte_test_state_read LTE_STATE_CIPCLOSE\r");

                    if (rx.indexOf("+CIPRXGET: 1") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ATE;
                        lte_test_state_read = LTE_STATE_ATE;
                    }
                    else if (rx.indexOf("CONNECT FAIL") != -1)
                    {
                        lte_test_state_write = LTE_STATE_ERROR;
                        lte_test_state_read = LTE_STATE_ERROR;
                    }
                    break;
                }
                case LTE_STATE_WAITING:
                {
                    break;
                }
                case LTE_STATE_ERROR:
                {
                    // error_count++;

                    // lte_test_state_write = LTE_STATE_CIPSHUT;
                    // lte_test_state_read = LTE_STATE_CIPSHUT;

                    // if (error_count == 4)
                    // {
                    //     error_count = 0;
                    //     printf("JIGA ISCA GSM FAIL. TRIED 3 TIMES");
                    //     lte_test_done = true;
                    //     xTaskNotify(m_otp_task, 0, eSetValueWithOverwrite);
                    // }
                    break;
                }
                case LTE_STATE_DONE:
                {
                    printf("JIGA ISCA GSM OK\r\n");
                    lte_test_state_write = LTE_STATE_POWER_OFF;
                    lte_test_state_read = LTE_STATE_POWER_OFF;
                    break;
                }
                case LTE_STATE_POWER_OFF:
                {
                    if (rx.indexOf("NORMAL POWER DOWN") != -1)
                        printf("POWER DOWN SUCCESS\r\n");

                    // power_off_modem();
                    lte_test_done = true;
                    esp_timer_stop(lte_timer);
                    esp_timer_delete(lte_timer);
                }
            } // switch read end
        }

        // if (Serial1.available())
        // {
        //     // Serial.write(Serial1.read());
        //     rx = Serial1.readString();
        //     // rx.trim();
        //     printf(rx);
        // }

        vTaskDelay(10);
    } // while end

    if (lte_timer_up)
    {
        printf("JIGA ISCA GSM TIMEOUT\r\n");
    }
}
// extern "C" void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

//     A76XX::A76XX *modem = new A76XX::A76XX(A7683_RXD_PIN, A7683_TXD_PIN, A7683_PWRKEY, A7683_POWER_ON, A7683_NETLIGHT);

//     modem->a76xx_power_on();
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
//     // modem->a76xx_power_on();
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
//     // modem->a76xx_power_on();
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
//     // modem->a76xx_power_on();
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);
//     // modem->a76xx_power_on();
//     // vTaskDelay(2000 / portTICK_PERIOD_MS);

//     uart_config();

//     xTaskCreate(rx_task, "update_task", 5120, NULL, 1, &rx_task_handle);

//     char* test_str = "AT\r";
//     while (1)
//     {
//         uart_write_bytes(UART_NUM_1, (const char*)test_str, strlen(test_str));
//         ESP_LOGW("main loop", "SENT AT\r\n");
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

esp_err_t send_at(const char *cmd, size_t size, char *out, const char *pass, const char *fail, int timeout)
{
    uart_write_bytes(UART_NUM_1, cmd, size);

    ESP_LOGW("send_at", "Sent to modem: %s", cmd);

    ModemUartRx_t r;
    BaseType_t ret;

    ret = xQueueReceive(xQueueModemRx, &r, timeout / portTICK_PERIOD_MS);

    ESP_LOGW("send_at", "Received from modem: %s", r.rx);

    memcpy(out, r.rx, r.len);

    if (strstr(r.rx, fail) != NULL)
        return ESP_FAIL;
    else if (strstr(r.rx, pass) != NULL)
        return ESP_OK;
    else if (ret == pdFALSE)
        return ESP_ERR_TIMEOUT;
    else
        return ESP_ERR_INVALID_ARG;
}

void power_on()
{
    const char *tag = "POWER ON";
    A76XX::A76XX *modem = new A76XX::A76XX(A7683_RXD_PIN, A7683_TXD_PIN, A7683_PWRKEY, A7683_POWER_ON, A7683_NETLIGHT);

    modem->a76xx_power_on();

    const esp_timer_create_args_t args = {
        .callback = &lte_timer_callback,
        .name = "lte_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &lte_timer));

    if(esp_timer_is_active(lte_timer))
        esp_timer_stop(lte_timer);

    lte_timer_up = false;

    timer_set_timer(lte_timer, 20000000); // sets timer for 20s for gsm testing

    ModemUartRx_t r;
    bool registered = false;
    bool rx_ok = false;

    while (!lte_timer_up && !registered)
    {
        if (xQueueReceive(xQueueModemRx, &r, 15000 / portTICK_PERIOD_MS) == pdFALSE)
        {
            lte_timer_up = true;
            
            if (rx_ok == false)
                ESP_LOGW(tag, "NO DATA RECEIVED AFTER POWER ON");
            else
                ESP_LOGW(tag, "REGISTERING TIMEOUT");
        }
        else
        {
            rx_ok = true;

            ESP_LOGW(tag, "Received from queue: %s", r.rx);

            if (strstr(r.rx, "+CGEV: ME PDN DEACT 8") != NULL)
            {
                registered = true;
                ESP_LOGW(tag, "Registered");
            }
        }

        vTaskDelay(10);
        // switch (state)
        // {

        // }
    }
}

void setup()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // err = print_what_saved();
    // if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    rg = new LedBicolor();
    rg->begin(PIN_LED_STATUS_RED, PIN_LED_STATUS_GREEN);
    rg->color(0x0000);

    // A76XX::A76XX *modem = new A76XX::A76XX(A7683_RXD_PIN, A7683_TXD_PIN, A7683_PWRKEY, A7683_POWER_ON, A7683_NETLIGHT);

    // modem->a76xx_power_on();

    xQueueModemRx = xQueueCreate(20, sizeof(ModemUartRx_t));

    xTaskCreate(rx_task, "rx_task", 5120, NULL, 1, &rx_task_handle);

    power_on();

    char* str = "AT+SIMEI?\r";
    char out[512];

    err = send_at(str, strlen(str), out, "OK", "ERROR", 30000);
    
    ESP_LOGW("test.cpp", "%s", esp_err_to_name(err));

    // xTaskCreate(tx_task, "tx_task", 5120, NULL, 1, &tx_task_handle);


    // Serial.begin(115200);
    // Serial1.begin(115200, SERIAL_8N1, A7683_RXD_PIN, A7683_TXD_PIN);

    // test_lte();
    // test_lbs();

    // xTaskCreatePinnedToCore(statusTask, "statusTask", 4096, NULL, 5, &status_task_handle, 0);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(10));
}