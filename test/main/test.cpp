#include "sdkconfig.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "LedBicolor.hpp"
#include "a76xx.hpp"

#define A7683_TXD_PIN         14
#define A7683_RXD_PIN         34
#define A7683_PWRKEY          25
#define A7683_POWER_ON        32
#define A7683_NETLIGHT        15
#define PIN_LED_STATUS_RED  GPIO_NUM_33
#define PIN_LED_STATUS_GREEN GPIO_NUM_25

bool can_send = false;
bool lte_timer_up = false;
static esp_timer_handle_t lte_timer;

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
    LTE_STATE_CIPCLOSE,
    LTE_STATE_CIPOPEN,
    LTE_STATE_CIPRXGET,
    LTE_STATE_CIPSEND,
    LTE_STATE_RESTART,
    LTE_STATE_MSG,
    LTE_STATE_RECEIVE,
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

void test_lte(A76XX *m)
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
    uint8_t error_count = 0;

    if(esp_timer_is_active(lte_timer))
        esp_timer_stop(lte_timer);

    lte_timer_up = false;

    esp_err_t err = ESP_OK;

    while (!lte_test_done && !lte_timer_up)
    {
        switch(lte_test_state_write)
        {
            case LTE_STATE_ATE:
            {
                m->set_echo_mode();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                lte_test_state_write = LTE_STATE_CREG;
                break;
            }
            case LTE_STATE_CREG:
            {
                err = m->check_registered();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CGREG;
                break;
            }
            case LTE_STATE_CGREG:
            {
                err = m->check_registration_status();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CMEE;
                break;
            }
            case LTE_STATE_CMEE:
            {
                err = m->set_error_report_numeric();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_NETCLOSE;
                break;
            }
            // TIMER
            case LTE_STATE_NETCLOSE:
            {
                err = m->stop_socket();
                timer_set_timer(lte_timer, 120000000); // sets timer for 15s for gsm testing
                // if (err != ESP_OK)
                //     lte_test_state_write = LTE_STATE_ERROR;
                // else
                lte_test_state_write = LTE_STATE_CGDCONT_1;
                break;
            }
            case LTE_STATE_CGDCONT_1:
            {
                err = m->set_pdp_context();
                timer_set_timer(lte_timer, 9000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CGDCONT_2;
                break;
            }
            case LTE_STATE_CGDCONT_2:
            {
                err = m->get_pdp_context();
                timer_set_timer(lte_timer, 75000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CSOCKSETPN;
                break;
            }
            case LTE_STATE_CSOCKSETPN:
            {
                err = m->set_pdp_context_active();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CIPRXGET;
                break;
            }
            case LTE_STATE_CIPRXGET:
            {
                err = m->set_retrieve_data_mode();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CIPMODE;
                break;
            }
            case LTE_STATE_CIPMODE:
            {
                err = m->set_tcpip_mode();
                timer_set_timer(lte_timer, 30000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_NETOPEN;
                break;
            }
            case LTE_STATE_NETOPEN:
            {
                err = m->start_socket_service();
                timer_set_timer(lte_timer, 65000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_IPADDR;
                break;
            }
            case LTE_STATE_IPADDR:
            {
                err = m->get_socket_ip();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CIPCLOSE;
                break;
            }
            case LTE_STATE_CIPCLOSE:
            {
                err = m->close_socket();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CIPOPEN;
                break;
            }
            case LTE_STATE_CIPOPEN:
            {
                err = m->tcp_connect();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_CIPSEND;
                break;
            }
            case LTE_STATE_CIPSEND:
            {
                err = m->tcp_send();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_MSG;
                break;
            }
            case LTE_STATE_RESTART:
            {
                err = m->close_socket();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_ATE;
                break;
            }
            case LTE_STATE_MSG:
            {
                char* str = "TEST\r";
                char out[512];
                // err = m->send_cmd(str, strlen(str), out, "+CIPSEND:", "ERROR", 30000);
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_RECEIVE;
    
                break;
            }
            case LTE_STATE_RECEIVE:
            {
                err = m->tcp_receive();
                timer_set_timer(lte_timer, 15000000); // sets timer for 15s for gsm testing
                
                if (err != ESP_OK)
                    lte_test_state_write = LTE_STATE_ERROR;
                else
                    lte_test_state_write = LTE_STATE_DONE;
    
                break;
            }
            case LTE_STATE_WAITING:
            {
                break;
            }
            case LTE_STATE_ERROR:
            {
                lte_test_state_write = LTE_STATE_RESTART;

                error_count++;
                if (error_count == 3)
                {
                    lte_timer_up = false;
                    error_count = 0;
                    printf("JIGA ISCA GSM FAIL. TRIED 3 TIMES\r\n");
                    lte_test_state_write = LTE_STATE_POWER_OFF;
                }
                break;
            }
            case LTE_STATE_DONE:
            {
                lte_test_state_write = LTE_STATE_POWER_OFF;
                break;
            }
            case LTE_STATE_POWER_OFF:
            {
                char* str = "AT+CPOF\r";
                char out[512];
                // err = m->send_cmd(str, strlen(str), out, "OK", "ERROR", 30000);
                timer_set_timer(lte_timer, 60000000); // sets timer for 15s for gsm testing
                
                if (err == ESP_OK)
                {
                    lte_test_done = true;
                    esp_timer_stop(lte_timer);
                    esp_timer_delete(lte_timer);
                }
                break;
            }
        } // switch write end

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    } // while end

    if (lte_timer_up) {
        printf("JIGA ISCA GSM TIMEOUT\r\n");
    }
}

void on_log_received(const std::string &log)
{
    ESP_LOGW("MODEM LOG", "%s", log.c_str());
}

void on_state_changed(const ModemStatus_t &state)
{
    ESP_LOGW("MODEM STATE", "STATE = %d", state);

    if (state == 2)
        can_send = true;
}

extern "C" void app_main(void)
{
    A76XX *modem = new A76XX(A7683_RXD_PIN, A7683_TXD_PIN, A7683_PWRKEY, A7683_POWER_ON, A7683_NETLIGHT, UART_NUM_1, 115200);

    modem->log_callback(on_log_received);
    modem->status_callback(on_state_changed);

    modem->power_on();

    while (!can_send)
        vTaskDelay(10);

    test_lte(modem);

    modem->power_off();

    while (1)
    {
        // ESP_LOGW("main loop", "ALIVE\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}