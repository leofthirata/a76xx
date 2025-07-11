#pragma once

#include <vector>
#include <string>
#include <tuple>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_timer.h"

typedef enum
{
    MODEM_STATE_OFF,
    MODEM_STATE_POWERING,
    MODEM_STATE_SEARCHING,
    MODEM_STATE_REGISTERED,
    MODEM_STATE_DETACH,
    MODEM_STATE_ATTACH,
    MODEM_STATE_NETWORK_CONTEXT_DEACT,
    MODEM_STATE_NETWORK_CLOSED,
} ModemStatus_t;

struct LBS {
    std::string lat;
    std::string lon;
    std::string date;
    std::string time;
};

using status_handle_t = void (*)(const ModemStatus_t &);
using log_handle_t = void (*)(const std::string &);

class A76XX
{
public:
    A76XX(int rxd_pin, int txd_pin, int pwrkey_pin, int power_on_pin, int netlight_pin, int uart, int baudrate);
    ~A76XX();

    void status_callback(status_handle_t s);
    void log_callback(log_handle_t s);
    
    void power_on();
    esp_err_t power_off();

    bool can_send();
    bool get_simcard() const;
    bool get_registered() const;
    bool get_on() const;

    void get_last_err(std::tuple<std::string, std::string> *out) const;

    esp_err_t send_data();
    esp_err_t send_data_and_receive();
    esp_err_t receive_data();
    esp_err_t update_position();

    LBS lbs {};

private:
    esp_err_t send_cmd(const std::string &cmd, std::string *out, const std::string &pass, const std::string &fail, const int &timeout);
    esp_err_t check_simcard();
    esp_err_t get_imei();
    esp_err_t get_signal_quality();
    esp_err_t get_operator_name();
    esp_err_t get_clock();
    esp_err_t set_echo_mode();
    esp_err_t check_gsm_network();
    esp_err_t check_gprs_lte_network();
    esp_err_t check_ue_info();

    esp_err_t check_network_and_simcard();
    esp_err_t pdp_context_init();
    esp_err_t connect_to_server();
    esp_err_t connect_to_server_rx_mode();
    esp_err_t disconnect_from_server();

    esp_err_t set_error_report_numeric();
    esp_err_t stop_socket_service();
    esp_err_t set_pdp_context();
    esp_err_t check_pdp_context();
    esp_err_t set_pdp_context_active();
    esp_err_t set_retrieve_data_mode();
    esp_err_t set_tcpip_mode();
    esp_err_t start_socket_service();
    esp_err_t get_socket_ip();
    esp_err_t close_socket();
    esp_err_t tcp_connect(const std::string &pass);
    esp_err_t tcp_send();
    esp_err_t tcp_receive();

    esp_err_t get_lbs();
    esp_err_t get_date_time();

    gpio_num_t m_power_on_pin;
    gpio_num_t m_pwrkey_pin;
    gpio_num_t m_netlight_pin;
    gpio_num_t m_rxd_pin;
    gpio_num_t m_txd_pin;

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

    std::tuple<std::string, std::string> last_cmd_fail {"", ""};

    bool m_power_up;
    bool m_send_cmd_up;
    bool m_just_turned_on;
    bool m_cmd_sent;
    bool m_on;
    bool m_simcard;
    bool m_registered {false};

    status_handle_t m_s;
    log_handle_t m_l;

    TaskHandle_t m_netlight_task_handle;
    TaskHandle_t m_rx_task_handle;
    TaskHandle_t m_power_task_handle;

    QueueHandle_t xQueueModemRx;

    BaseType_t xHigherPriorityTaskWoken;

    int rssi {0};
    int rssi_dbm {0};
    int ber {0};
    std::string ip {""};
    std::string nw_operator {""};
    std::string nw_tech {""};

    friend void netlight_task(void *args);
    friend void power_task(void *args);
    friend void rx_task(void *args);

    friend void IRAM_ATTR a76xx_netlight_isr(void *args);
    friend void timer_callback(void *args);
    friend void power_up_timer_callback(void *args);

    void netlight_config();
    void pwrkey_config() const;
    void uart_config() const;
    void set_state(ModemStatus_t state);
};