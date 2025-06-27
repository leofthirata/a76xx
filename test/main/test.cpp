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

    // auto banding?

    // query sim card status
    m->check_simcard();

    m->get_signal_quality();

    // query gsm network
    m->check_gsm_network();
    // query gprs/lte network in 90s
    m->check_gprs_lte_network();

    // activate pdp context
    // AT+CGDCONT and AT+CGACT
    // query ip address of pdp context AT+CGPADDR
    m->set_pdp_context();
    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    m->get_pdp_context();
    m->set_pdp_context_active();

    m->set_retrieve_data_mode();

    // set tcp/ip mode
    m->set_tcpip_mode();

    // activate pdp context
    m->activate_pdp_context();

    // get ip addr
    m->get_socket_ip();

    // establish connection
    m->tcp_connect();

    m->tcp_receive();

    m->tcp_send();

    // // close socket
    m->close_socket();

    // // deactivate pdp context
    m->deactivate_pdp_context();
}

void on_log_received(const std::string &log)
{
    ESP_LOGW("MODEM LOG", "%s", log.c_str());
}

void on_state_changed(const ModemStatus_t &state)
{
    ESP_LOGW("MODEM STATE", "STATE = %d", state);

    if (state == 3)
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

    while (!modem->can_send())
        vTaskDelay(500 / portTICK_PERIOD_MS);

    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    test_lte(modem);

    std::string lat;
    std::string lon;
    modem->get_lbs(&lat, &lon);

    ESP_LOGI("main", "%s", lat.c_str());
    ESP_LOGI("main", "%s", lon.c_str());

    modem->power_off();

    while (1)
    {
        // ESP_LOGW("main loop", "ALIVE\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}