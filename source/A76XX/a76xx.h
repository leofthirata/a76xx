#ifndef _A76XX_H_
#define _A76XX_H_

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

namespace MODEM
{

typedef enum
{
    MODEM_STATE_OFF,
    MODEM_STATE_POWERING,
    MODEM_STATE_SEARCHING,
    MODEM_STATE_REGISTERED,
    MODEM_STATE_DETACH,
    MODEM_STATE_ATTACH,
} ModemStatus_t;

using status_handle_t = void (*)(ModemStatus_t);
using log_handle_t = void (*)(const char *);

class A76XX
{
public:
    A76XX(int rxd_pin, int txd_pin, int pwrkey_pin, int power_on_pin, int netlight_pin, int uart, int baudrate);
    ~A76XX();

    void status_callback(status_handle_t s);
    void log_callback(log_handle_t s);
    void power_on();
    void power_off();
    bool get_simcard();
    bool get_registered();
    bool get_on();
    esp_err_t send_cmd(const char *cmd, size_t size, char *out, const char *pass, const char *fail, int timeout);

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

    friend void IRAM_ATTR a76xx_netlight_isr(void *arg);
    friend void timer_callback(void *arg);
    friend void power_up_timer_callback(void *arg);
    friend void send_cmd_timer_callback(void *arg);

    void netlight_config();
    void pwrkey_config();
    void uart_config();
    void set_state(ModemStatus_t state);
    void timer_set_timer(esp_timer_handle_t timer, uint64_t timeout);
};

} // MODEM namespace

#endif // _A76XX_H_
