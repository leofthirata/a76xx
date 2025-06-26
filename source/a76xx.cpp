#include "a76xx.hpp"

#define HIGH 1
#define LOW 0

const char *TAG = "a76xx.cpp";

typedef struct
{
    int len;
    char rx[512];
} ModemUartRx_t;

void timer_callback(void *args);
void power_up_timer_callback(void *args);

void netlight_task(void *args);
void power_task(void *args);
void rx_task(void *args);

void IRAM_ATTR a76xx_netlight_isr(void *args)
{
    A76XX *modem = static_cast<A76XX *>(args);

    static unsigned long pulseStartPositiveTime, pulseStartNegativeTime; // Start time variable
    unsigned long now = esp_timer_get_time();

    if (gpio_get_level((gpio_num_t)modem->m_netlight_pin) == HIGH) { // If the change was a RISING edge
        pulseStartPositiveTime = now; // Store the start time (in microseconds)
        modem->isrNegativeWidth = now - pulseStartNegativeTime;
    }
    else { // If the change was a FALLING edge
        modem->isrPositiveWidth = now - pulseStartPositiveTime; // Calculate the pulsewidth
        pulseStartNegativeTime = now;
        modem->isrLastPulseStartNegativeTime = now;
    }

    xTaskNotifyFromISR(modem->m_netlight_task_handle, 0x01, eSetBits, &modem->xHigherPriorityTaskWoken);
}

void timer_callback(void *args)
{
    A76XX *modem = static_cast<A76XX *>(args);
    xTaskNotify(modem->m_power_task_handle, 0x01, eSetBits);
}

void power_up_timer_callback(void *args)
{
    A76XX *modem = static_cast<A76XX *>(args);
    modem->m_power_up = true;
    modem->set_state(MODEM_STATE_SEARCHING);
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
    
    xQueueModemRx = xQueueCreate(20, sizeof(ModemUartRx_t));
    if (xQueueModemRx == 0)
        ESP_LOGE(TAG, "xQueueCreate fail to create xQueueModemRx\r\n");

    xTaskCreate(netlight_task, "netlight_task", 1024, this, 1, &m_netlight_task_handle);
    xTaskCreate(power_task, "power_task", 1024, this, 1, &m_power_task_handle);
    xTaskCreate(rx_task, "rx_task", 3072, this, 2, &m_rx_task_handle);
}

A76XX::~A76XX()
{
}

void netlight_task(void *args)
{
    A76XX *modem = static_cast<A76XX *>(args);

    int64_t now = 0;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        portDISABLE_INTERRUPTS();
        modem->positivePulseWidth = modem->isrPositiveWidth;
        modem->negativePulseWidth = modem->isrNegativeWidth;
        modem->lastPulseStartNegativeTime = modem->isrLastPulseStartNegativeTime;
        portENABLE_INTERRUPTS();
        now = esp_timer_get_time();
        if ((now - modem->lastPulseStartNegativeTime) / 1000 > 300) {
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
    A76XX *modem = static_cast<A76XX *>(args);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (gpio_get_level((gpio_num_t)modem->m_netlight_pin) == 0) {
            modem->set_state(MODEM_STATE_OFF);
        }
        else {
            if (modem->m_power_up)
                modem->set_state(MODEM_STATE_SEARCHING);
        }
        vTaskDelay(10);
    }
}

void rx_task(void *args) // UART Receive Task
{
    A76XX *modem = static_cast<A76XX *>(args);
    
    const char *TAG = "modem serial rx task";
    ModemUartRx_t response;

    ESP_LOGW(TAG, "RX TASK BEGIN\r\n");

    while (1)
    {
        response.len = uart_read_bytes(modem->m_uart, response.rx, 512, (100 / portTICK_PERIOD_MS));
        if (response.len > 0) {
            response.rx[response.len] = 0;
            std::string rsp {response.rx[response.len]};

            if (modem->m_just_turned_on) {
                modem->set_state(MODEM_STATE_POWERING);
                modem->m_just_turned_on = false;
            }
            // 5.2.15 AT+CGEREP GPRS event reporting
            if (rsp.find("+CPIN: READY") != std::string::npos)
                modem->set_state(MODEM_STATE_ATTACH);
            else if (rsp.find("+CPIN: SIM REMOVED") != std::string::npos)
                modem->set_state(MODEM_STATE_DETACH);
            else if (rsp.find("+CGEV: NW PDN ACT") != std::string::npos)
                modem->set_state(MODEM_STATE_REGISTERED);

            if (!modem->m_cmd_sent)
                modem->m_l(rsp);

            int ret = xQueueSend(modem->xQueueModemRx, &response, (TickType_t)0);
            if (ret != pdTRUE)
                ESP_LOGE(TAG, "RX TASK Failed to queue response to xQueueModemRx\r\n");
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

esp_err_t A76XX::send_cmd(const std::string &cmd, std::string *out, const std::string &pass, const std::string &fail, const int &timeout)
{
    if (!m_on) {
        ESP_LOGW("send_cmd", "Modem is offline. Turn on the modem first before sending any AT command.");
        return ESP_ERR_INVALID_STATE;
    }

    if (!m_power_up) {
        ESP_LOGW("send_cmd", "Modem is powering up. Wait until power up is complete");
        return ESP_ERR_INVALID_STATE;
    }

    m_cmd_sent = true;

    uart_write_bytes(m_uart, cmd.c_str(), cmd.size());

    ESP_LOGW("send_cmd", "Sent to modem: %s", cmd.c_str());

    ModemUartRx_t r;

    xQueueReset(xQueueModemRx);

    int ok = 0;
    while (ok == 0) {
        BaseType_t ret = xQueueReceive(xQueueModemRx, &r, timeout / portTICK_PERIOD_MS);
        if (ret != pdTRUE) {
            ok = -2;
            return ESP_ERR_TIMEOUT;
        }

        ESP_LOGI("send_cmd", "Received from modem: %s", r.rx);

        *out = r.rx;

        m_cmd_sent = false;

        ok = out->find(fail) != std::string::npos ? -1 : 1;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return ok == 1 ? ESP_OK : ESP_FAIL;
}

void A76XX::set_state(ModemStatus_t state)
{
    m_state = state;

    if (m_state == m_prev_state)
        return;

    m_s(m_state);
    m_prev_state = m_state;

    switch (m_state) {
        case MODEM_STATE_OFF: {
            m_on = false;
            break;
        }
        case MODEM_STATE_POWERING: {
            m_on = true;
            m_power_up = false;
            timer_set_timer(m_power_up_timer, 10000000);
            break;
        }
        case MODEM_STATE_SEARCHING: {
            m_on = true;
            m_registered = false;
            break;
        }
        case MODEM_STATE_REGISTERED: {
            m_on = true;
            m_registered = true;
            break;
        }
        case MODEM_STATE_DETACH: {
            m_on = true;
            m_simcard = false;
            break;
        }
        case MODEM_STATE_ATTACH: {
            m_on = true;
            m_simcard = false;
            break;
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
    io_conf.mode = GPIO_MODE_OUTPUT;                                          // set as output modem->
    io_conf.pin_bit_mask = (1ULL << m_power_on_pin) | (1ULL << m_pwrkey_pin); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                             // disable pull-down modem->
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                 // disable pull-up modem->

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

esp_err_t A76XX::power_off()
{
    const std::string cmd = "AT+CPOF\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    esp_err_t ret = send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
    
    gpio_set_level((gpio_num_t)m_power_on_pin, false);

    return ret;
}

esp_err_t A76XX::get_imei()
{
    const std::string cmd = "AT+CGSN\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::get_signal_quality()
{
    const std::string cmd = "AT+CSQ\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::get_operator_name()
{
    const std::string cmd = "AT+COPS?\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::get_clock()
{
    const std::string cmd = "AT+CCLK?\r";
    
    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::set_echo_mode()
{
    const std::string cmd = "ATE1\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::check_registered()
{
    const std::string cmd = "AT+CREG?\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+CREG: 0,1", "ERROR", 30000);
}

esp_err_t A76XX::check_registration_status()
{
    const std::string cmd = "AT+CGREG?\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+CGREG: 0,1", "ERROR", 30000);
}

esp_err_t A76XX::set_error_report_numeric()
{
    const std::string cmd = "AT+CMEE=1\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::stop_socket()
{
    const std::string cmd = "AT+NETCLOSE\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "", "", 30000);
}

esp_err_t A76XX::set_pdp_context()
{
    const std::string cmd = "AT+CGDCONT=1,\"IP\",\"simplepm.algar.br\",\"0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0\",0,0\r";
    
    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::get_pdp_context()
{
    const std::string cmd = "AT+CGDCONT?\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::set_pdp_context_active()
{
    const std::string cmd = "AT+CSOCKSETPN=1\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::set_retrieve_data_mode()
{
    const std::string cmd = "AT+CIPRXGET=1\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::set_tcpip_mode()
{
    const std::string cmd = "AT+CIPMODE=0\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "OK", "ERROR", 30000);
}

esp_err_t A76XX::start_socket_service()
{
    const std::string cmd = "AT+NETOPEN\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+NETOPEN: 0", "ERROR", 30000);
}

esp_err_t A76XX::get_socket_ip()
{
    const std::string cmd = "AT+IPADDR\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+IPADDR:", "ERROR", 30000);
}

esp_err_t A76XX::close_socket()
{
    const std::string cmd = "AT+CIPCLOSE=0\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+CIPCLOSE", "DUMMY", 30000);
}

esp_err_t A76XX::tcp_connect()
{
    const std::string cmd = "AT+CIPOPEN=0,\"TCP\",\"0.tcp.sa.ngrok.io\",11197\r";
    
    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "+CIPRXGET: 1,0", "ERROR", 30000);
}

esp_err_t A76XX::tcp_send()
{
    const std::string cmd = "AT+CIPSEND=0,5\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;
    int ret = send_cmd(cmd.c_str(), &out, "", "ERROR", 30000);
    if (ret != ESP_OK)
        return ret;

    return send_cmd(cmd.c_str(), &out, "+CIPSEND:", "ERROR", 30000);
}

esp_err_t A76XX::tcp_receive()
{
    const std::string cmd = "AT+CIPRXGET=2,0\r";

    ESP_LOGW(TAG, "%s", cmd.c_str());
    std::string out;

    return send_cmd(cmd.c_str(), &out, "", "DUMMY", 30000);
}