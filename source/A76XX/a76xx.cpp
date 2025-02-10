#include "a76xx.h"

#define HIGH 1
#define LOW 0

#define TAG "A76XX"

typedef struct
{
    int len;
    char rx[512];
} ModemUartRx_t;

namespace MODEM
{

void timer_callback(void *arg);
void power_up_timer_callback(void *arg);
void send_cmd_timer_callback(void *arg);

void IRAM_ATTR a76xx_netlight_isr(void *arg);

void netlight_task(void *args);
void power_task(void *args);
void rx_task(void *args);

void IRAM_ATTR a76xx_netlight_isr(void *arg)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(arg);

    static unsigned long pulseStartPositiveTime, pulseStartNegativeTime; // Start time variable
    unsigned long now = esp_timer_get_time();

    if (gpio_get_level((gpio_num_t)modem->m_netlight_pin) == HIGH) // If the change was a RISING edge
    {
        pulseStartPositiveTime = now; // Store the start time (in microseconds)
        modem->isrNegativeWidth = now - pulseStartNegativeTime;
    }
    else // If the change was a FALLING edge
    {
        modem->isrPositiveWidth = now - pulseStartPositiveTime; // Calculate the pulsewidth
        pulseStartNegativeTime = now;
        modem->isrLastPulseStartNegativeTime = now;
    }

    xTaskNotifyFromISR(modem->m_netlight_task_handle, 0x01, eSetBits, &modem->xHigherPriorityTaskWoken);
}

void timer_callback(void *arg)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(arg);
    xTaskNotify(modem->m_power_task_handle, 0x01, eSetBits);
}

void power_up_timer_callback(void *arg)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(arg);
    modem->m_power_up = true;
    modem->set_state(MODEM_STATE_SEARCHING);
}

void send_cmd_timer_callback(void *arg)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(arg);
    modem->m_send_cmd_up = true;
}

void A76XX::status_callback(status_handle_t s)
{
    m_s = s;
}

void A76XX::log_callback(log_handle_t l)
{
    m_l = l;
}

A76XX::A76XX(int rxd_pin, int txd_pin, int pwrkey_pin, int power_on_pin, int netlight_pin, int uart, int baudrate)
{
    m_rxd_pin = rxd_pin;
    m_txd_pin = txd_pin;
    m_pwrkey_pin = pwrkey_pin;
    m_power_on_pin = power_on_pin;
    m_netlight_pin = netlight_pin;
    m_uart = uart;
    m_baurate = baudrate;
    m_power_up = false;
    m_send_cmd_up = false;
    m_just_turned_on = true;
    m_cmd_sent = false;
    m_on = false;
    m_registered = false;
    m_simcard = false;
    m_s = NULL;
    m_l = NULL;

    isrPositiveWidth = 0; 
    isrNegativeWidth = 0; 
    isrLastPulseStartNegativeTime = 0; 
    isrLastPulseStartPositiveTime = 0;
    positivePulseWidth = 0;
    negativePulseWidth = 0;
    lastPulseStartNegativeTime = 0;
    lastPulseStartPositiveTime = 0;

    uart_config();

    pwrkey_config();
    netlight_config();

    const esp_timer_create_args_t args = {
        .callback = &timer_callback,
        .arg = this,
        .name = "timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &m_timer));

    const esp_timer_create_args_t args2 = {
        .callback = &power_up_timer_callback,
        .arg = this,
        .name = "power_up_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args2, &m_power_up_timer));

    const esp_timer_create_args_t args3 = {
        .callback = &send_cmd_timer_callback,
        .arg = this,
        .name = "send_cmd_timer"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&args3, &m_send_cmd_timer));
    
    xQueueModemRx = xQueueCreate(20, sizeof(ModemUartRx_t));

    xTaskCreate(netlight_task, "netlight_task", 1024, this, 1, &m_netlight_task_handle);
    xTaskCreate(power_task, "power_task", 1024, this, 1, &m_power_task_handle);
    xTaskCreate(rx_task, "rx_task", 3072, this, 2, &m_rx_task_handle);
}

A76XX::~A76XX()
{
}

void A76XX::power_on()
{
    gpio_set_level((gpio_num_t)m_power_on_pin, true);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)m_pwrkey_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)m_pwrkey_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)m_pwrkey_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level((gpio_num_t)m_pwrkey_pin, 0);

    m_just_turned_on = true;
}

void A76XX::power_off()
{
    char* str = "AT+CPOF\r";
    char out[512];

    esp_err_t err = send_cmd(str, strlen(str), out, "OK", "ERROR", 30000);
    
    gpio_set_level((gpio_num_t)m_power_on_pin, false);
}

void netlight_task(void *args)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(args);

    int16_t count = 0;
    int64_t now = 0;
    while (1)
    {
        auto event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        portDISABLE_INTERRUPTS();
        modem->positivePulseWidth = modem->isrPositiveWidth;
        modem->negativePulseWidth = modem->isrNegativeWidth;
        modem->lastPulseStartNegativeTime = modem->isrLastPulseStartNegativeTime;
        portENABLE_INTERRUPTS();
        now = esp_timer_get_time();
        if ((now - modem->lastPulseStartNegativeTime) / 1000 > 300)
        {
            modem->positivePulseWidth = 0;
            modem->negativePulseWidth = 0;
        }

        modem->timer_set_timer(modem->m_timer, 5000000);
        // 200ms +- 10%
        if (modem->negativePulseWidth / 1000 < 240 && modem->negativePulseWidth / 1000 > 160)
            modem->set_state(MODEM_STATE_REGISTERED);
    }
}

void power_task(void *args)
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(args);

    ModemUartRx_t r;
    bool rx_ok = false;

    while (1)
    {
        auto event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (gpio_get_level((gpio_num_t)modem->m_netlight_pin) == 0)
            modem->set_state(MODEM_STATE_OFF);
        else
        {
            if (modem->m_power_up)
                modem->set_state(MODEM_STATE_SEARCHING);
        }
        vTaskDelay(10);
    }
}

void rx_task(void *args) // UART Receive Task
{
    MODEM::A76XX *modem = static_cast<MODEM::A76XX *>(args);
    
    static const char *tag = "modem serial task";
    ModemUartRx_t response;

    ESP_LOGW(tag, "RX TASK BEGIN\r\n");

    while (1)
    {
        response.len = uart_read_bytes(modem->m_uart, response.rx, 512, (100 / portTICK_PERIOD_MS));
        if (response.len > 0)
        {
            response.rx[response.len] = 0;

            if (modem->m_just_turned_on)
            {
                modem->set_state(MODEM_STATE_POWERING);
                modem->m_just_turned_on = false;
            }
            // 5.2.15 AT+CGEREP GPRS event reporting
            if (strstr(response.rx, "+CPIN: READY") != NULL)
                modem->set_state(MODEM_STATE_ATTACH);
            if (strstr(response.rx, "+CPIN: SIM REMOVED") != NULL)
                modem->set_state(MODEM_STATE_DETACH);
            if (strstr(response.rx, "+CGEV: NW PDN ACT") != NULL)
                modem->set_state(MODEM_STATE_REGISTERED);

            if (!modem->m_cmd_sent)
                modem->m_l(response.rx);

            xQueueSend(modem->xQueueModemRx, &response, NULL);
        }
        vTaskDelay(10);
    }
}

bool A76XX::get_simcard()
{
    return m_simcard;
}

bool A76XX::get_registered()
{
    return m_registered;
}

bool A76XX::get_on()
{
    return m_on;
}

esp_err_t A76XX::send_cmd(const char *cmd, size_t size, char *out, const char *pass, const char *fail, int timeout)
{
    if (!m_on)
    {
        ESP_LOGW("send_cmd", "Modem is offline. Turn on the modem first before sending any AT command.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!m_power_up)
    {
        ESP_LOGW("send_cmd", "Modem is powering up. Wait until power up is complete");
        return ESP_ERR_INVALID_STATE;
    }

    m_cmd_sent = true;

    uart_write_bytes(m_uart, cmd, size);

    ESP_LOGW("send_cmd", "Sent to modem: %s", cmd);

    ModemUartRx_t r;
    BaseType_t ret;

    ret = xQueueReset(xQueueModemRx);

    timer_set_timer(m_send_cmd_timer, timeout * 1000);

    int ok = 0;

    while (!m_send_cmd_up && ok == 0)
    {
        ret = xQueueReceive(xQueueModemRx, &r, timeout / portTICK_PERIOD_MS);

        ESP_LOGI("send_cmd", "Received from modem: %s", r.rx);

        memcpy(out, r.rx, r.len);

        m_cmd_sent = false;

        if (strstr(r.rx, fail) != NULL)
            ok = -1;
        else if (strstr(r.rx, pass) != NULL)
            ok = 1;
        else if (ret == pdFALSE)
            ok = -2;
    }

    esp_timer_stop(m_send_cmd_timer);

    if (ok == 1)
        return ESP_OK;
    else if (ok == -1)
        return ESP_FAIL;
    else
        return ESP_ERR_TIMEOUT;
}

void A76XX::set_state(ModemStatus_t state)
{
    m_state = state;

    if (m_state != m_prev_state)
    {
        m_s(m_state);
        m_prev_state = m_state;

        switch (m_state)
        {
            case MODEM_STATE_OFF:
            {
                m_on = false;
                break;
            }
            case MODEM_STATE_POWERING:
            {
                m_on = true;
                m_power_up = false;
                timer_set_timer(m_power_up_timer, 10000000);
                break;
            }
            case MODEM_STATE_SEARCHING:
            {
                m_on = true;
                m_registered = false;
                break;
            }
            case MODEM_STATE_REGISTERED:
            {
                m_on = true;
                m_registered = true;
                break;
            }
            case MODEM_STATE_DETACH:
            {
                m_on = true;
                m_simcard = false;
                break;
            }
            case MODEM_STATE_ATTACH:
            {
                m_on = true;
                m_simcard = false;
                break;
            }
        }
    }
}

void A76XX::timer_set_timer(esp_timer_handle_t timer, uint64_t timeout)
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    esp_timer_start_once(timer, timeout);
}

void A76XX::uart_config()
{
    uart_config_t config = {};
    config.baud_rate = m_baurate;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.rx_flow_ctrl_thresh = 122;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(m_uart, 127 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(m_uart, &config));
    ESP_ERROR_CHECK(uart_set_pin(m_uart, m_txd_pin, m_rxd_pin, -1, -1));
    ESP_ERROR_CHECK(uart_set_mode(m_uart, UART_MODE_UART));
    ESP_ERROR_CHECK(uart_set_rx_timeout(m_uart, 3));
}

void A76XX::pwrkey_config()
{
    gpio_config_t io_conf = {}; // zero-initialize the config structure.

    io_conf.intr_type = GPIO_INTR_DISABLE;                                    // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                          // set as output mode
    io_conf.pin_bit_mask = (1ULL << m_power_on_pin) | (1ULL << m_pwrkey_pin); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                             // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                 // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level((gpio_num_t)m_power_on_pin, false);
    gpio_set_level((gpio_num_t)m_pwrkey_pin, false);
}

void A76XX::netlight_config()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << m_netlight_pin);
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)m_netlight_pin, a76xx_netlight_isr, this);
}


} // MODEM namespace