#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <functional>
#include <string>

#include "esp_system.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include <esp_check.h>
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "driver/gpio.h"
#include "cxx_include/esp_modem_api.hpp"
#include "cxx_include/esp_modem_dce_factory.hpp"
#include "cxx_include/esp_modem_dce.hpp"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_dce_module.hpp"
#include "esp_modem_config.h"
#include "string.h"
#include "sys/time.h"
#include "esp_timer.h"

typedef enum
{
    MODEM_STATE_OFF,
    MODEM_STATE_POWERING,
    MODEM_STATE_SEARCHING,
    MODEM_STATE_REGISTERED,
    MODEM_STATE_DETACH,
    MODEM_STATE_ATTACH,
} ModemStatus_t;

using status_handle_t = void (*)(const ModemStatus_t &);
using log_handle_t = void (*)(const std::string &);

class A76XX
{
public:
    A76XX(int rxd_pin, int txd_pin, int pwrkey_pin, int power_on_pin, int netlight_pin, int uart, int baudrate);
    ~A76XX();

    void status_callback(status_handle_t s);
    void log_callback(log_handle_t s);
    
    esp_err_t send_cmd(const std::string &cmd, std::string *out, const std::string &pass, const std::string &fail, const int &timeout);

    esp_err_t get_lbs(std::string *lat, std::string *lon);
    esp_err_t get_date_time(std::string *date, std::string *time);

    bool can_send();
    void power_on();
    esp_err_t power_off();
    esp_err_t check_simcard();
    esp_err_t get_imei();
    esp_err_t get_signal_quality();
    esp_err_t get_operator_name();
    esp_err_t get_clock();
    esp_err_t set_echo_mode();
    esp_err_t check_gsm_network();
    esp_err_t check_gprs_lte_network();
    esp_err_t set_error_report_numeric();
    esp_err_t deactivate_pdp_context();
    esp_err_t set_pdp_context();
    esp_err_t get_pdp_context();
    esp_err_t set_pdp_context_active();
    esp_err_t set_retrieve_data_mode();
    esp_err_t set_tcpip_mode();
    esp_err_t activate_pdp_context();
    esp_err_t get_socket_ip();
    esp_err_t close_socket();
    esp_err_t tcp_connect();
    esp_err_t tcp_send();
    esp_err_t tcp_receive();

    bool get_simcard();
    bool get_registered();
    bool get_on();

private:
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t en_pin;

    int m_power_on_pin;
    int m_pwrkey_pin;
    int m_netlight_pin;
    int m_rxd_pin;
    int m_txd_pin;

    int m_uart;
    int m_baurate;

    volatile unsigned long isrPositiveWidth;
    volatile unsigned long isrNegativeWidth;
    volatile unsigned long isrLastPulseStartNegativeTime;
    volatile unsigned long isrLastPulseStartPositiveTime;
    unsigned long positivePulseWidth;
    unsigned long negativePulseWidth;
    unsigned long lastPulseStartNegativeTime;
    unsigned long lastPulseStartPositiveTime;

    ModemStatus_t m_state;
    ModemStatus_t m_prev_state;
    esp_timer_handle_t m_timer;
    esp_timer_handle_t m_power_up_timer;
    esp_timer_handle_t m_send_cmd_timer;

    bool m_power_up;
    bool m_send_cmd_up;
    bool m_just_turned_on;
    bool m_cmd_sent;
    bool m_on;
    bool m_registered;
    bool m_simcard;

    status_handle_t m_s;
    log_handle_t m_l;

    TaskHandle_t m_netlight_task_handle;
    TaskHandle_t m_rx_task_handle;
    TaskHandle_t m_power_task_handle;

    QueueHandle_t xQueueModemRx;

    BaseType_t xHigherPriorityTaskWoken;

    friend void netlight_task(void *args);
    friend void power_task(void *args);
    friend void rx_task(void *args);

    friend void IRAM_ATTR a76xx_netlight_isr(void *args);
    friend void timer_callback(void *args);
    friend void power_up_timer_callback(void *args);
    friend void send_cmd_timer_callback(void *args);

    void netlight_config();
    void pwrkey_config();
    void uart_config();
    void set_state(ModemStatus_t state);
    void timer_set_timer(esp_timer_handle_t timer, uint64_t timeout);
};