#include "a76xx.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/uart.h"

#define A7683_TXD_PIN         14
#define A7683_RXD_PIN         34
#define A7683_PWRKEY          25
#define A7683_POWER_ON        32
#define A7683_NETLIGHT        15

bool can_send = false;
int state = 7;

void handle_error(A76XX *modem);

void timer_set_timer(esp_timer_handle_t timer, uint64_t timeout)
{
    if (esp_timer_is_active(timer))
        esp_timer_stop(timer);

    esp_timer_start_once(timer, timeout);
}

void on_log_received(const std::string &log)
{
    ESP_LOGI("MODEM LOG", "%s", log.c_str());
}

void on_state_changed(const ModemStatus_t &state)
{
    ESP_LOGW("MODEM STATE", "STATE = %d", state);
    if (state == 2)
        can_send = true; 
}

void state_timer_callback(void *args)
{
    ESP_LOGW("STATE TIMER", "triggered");
    state = 0;
}

// TODO check heap before and after
// TODO check heap before and after deleting pointer
extern "C" void app_main(void)
{
    // esp_log_level_set("*", ESP_LOG_DEBUG);

    A76XX *modem = new A76XX(A7683_RXD_PIN, A7683_TXD_PIN, A7683_PWRKEY, A7683_POWER_ON, A7683_NETLIGHT, UART_NUM_2, 115200);

    modem->log_callback(on_log_received);
    modem->status_callback(on_state_changed);

    esp_timer_handle_t state_timer;

    const esp_timer_create_args_t args = {
        .callback = &state_timer_callback,
        .arg = nullptr,
        .name = "state_timer_callback"
    };

    ESP_ERROR_CHECK(esp_timer_create(&args, &state_timer));

    esp_timer_start_once(state_timer, 0);

    while (1) {
        switch (state) {
            case 0: {
                modem->power_on();

                while (!can_send)
                    vTaskDelay(pdMS_TO_TICKS(10));

                while (!modem->can_send())
                    vTaskDelay(500 / portTICK_PERIOD_MS);

                state = 2;
                break;
            }
            case 1: {
                int ret = modem->send_data_and_receive();
                state = ret == ESP_OK ? 4 : 5;
                break;
            }
            case 2: {
                int ret = modem->send_data();
                state = ret == ESP_OK ? 4 : 5;
                break;
            }
            case 3: {
                int ret = modem->receive_data();
                state = ret == ESP_OK ? 4 : 5;
                break;
            }
            case 4: {
                int ret = modem->update_position();
                if (ret == ESP_OK) {
                    ESP_LOGI("main", "%s", modem->lbs.lat.c_str());
                    ESP_LOGI("main", "%s", modem->lbs.lon.c_str());
                }
                state = 5;
                break;
            }
            case 5: {
                handle_error(modem);
                modem->power_off();

                state = 6;

                while (modem->get_on())
                    vTaskDelay(10);
                    
                esp_timer_start_once(state_timer, 60000000);
                break;
            }
            case 6: {

                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void handle_error(A76XX *modem)
{
    std::tuple<std::string, std::string> cmd_err;
                    
    modem->get_last_err(&cmd_err);
    ESP_LOGE("main", "cmd: %s", std::get<0>(cmd_err).c_str());
    ESP_LOGE("main", "err: %s", std::get<1>(cmd_err).c_str());
}