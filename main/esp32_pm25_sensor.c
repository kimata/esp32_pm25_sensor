#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "driver/mcpwm.h"
#include "driver/i2c.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "lwip/sockets.h"

#include "esp_attr.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_sleep.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include "cJSON.h"

#include <stdlib.h>
#include <string.h>

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

#define FLUENTD_IP          "192.168.2.20"  // IP address of Fluentd
#define FLUENTD_PORT        8888            // Port of FLuentd
#define FLUENTD_TAG         "/sensor"       // Fluentd tag
#define WIFI_HOSTNAME       "ESP32-indoor2"  // module's hostname
#define SENSE_INTERVAL      600            // sensing interval
#define WAIT_BEFORE_SLEEP   20             // sensing interval

#define WIFI_SSID "XXXXXXXX"            // WiFi SSID
#define WIFI_PASS "XXXXXXXX"            // WiFi Password

#define ADC_VREF        1114            // ADC calibration data

#define DUST_ADC_CH  ADC1_CHANNEL_4  // GPIO 32
#define DUST_LED_GPIO GPIO_NUM_26
#define DUST_AD_AVE_NUM 16
#define DUST_FAN_GPIO GPIO_NUM_14

#define DUST_CALIB_VOL  890 // Vs voltage (mV)
#define DUST_CALIB_TEMP 20  // temperature at the time of calibration

#define SHT3x_DEV_ADDR 0x88 // 8bit
#define SHT3x_CMD_MSB  0x24
#define SHT3x_CMD_LSB 0x00

#define I2C_SDA 33
#define I2C_SCL 25
#define I2C_FREQ 100000 // 100kHz

#define WIFI_CONNECT_TIMEOUT 20

#define TAG "air"

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#endif

typedef struct sht3x_sense_data {
    union {
        uint16_t temp;
        uint8_t temp_bytes[2];
    };
    uint8_t temp_crc;
    union {
        uint16_t humi;
        uint8_t humi_bytes[2];
    };
    uint8_t humi_crc;
} sht3x_sense_data_t;

typedef struct sht3x_sense_value {
    float temp;
    float humi;
} sht3x_sense_value_t;

typedef enum {
    wifi_connected,
    wifi_disconnected,
} wifi_status_t;

#define EXPECTED_RESPONSE "HTTP/1.1 200 OK"
#define REQUEST "POST http://" FLUENTD_IP FLUENTD_TAG " HTTP/1.0\r\n" \
    "Content-Type: application/x-www-form-urlencoded\r\n" \
    "Content-Length: %d\r\n" \
    "\r\n" \
    "json=%s"

static volatile wifi_status_t wifi_status = wifi_disconnected;
static uint32_t dust_ad_idx = 0;
static uint32_t dust_ad_list[300];

SemaphoreHandle_t sense_done = NULL;
esp_adc_cal_characteristics_t adc_char;

//////////////////////////////////////////////////////////////////////
// DN7C3CA006 (dust sensor)
static void IRAM_ATTR isr_handler()
{
    signed portBASE_TYPE task_woken;

    // clear interrupt
    MCPWM[MCPWM_UNIT_0]->int_clr.val = MCPWM[MCPWM_UNIT_0]->int_st.val;

    if (dust_ad_idx != ARRAY_SIZE(dust_ad_list)) {
        dust_ad_list[dust_ad_idx++] = adc1_to_voltage(DUST_ADC_CH, &adc_char);
    } else {
        xSemaphoreGiveFromISR(sense_done, &task_woken);
    }
}

static void dust_sense_start()
{
    xSemaphoreTake(sense_done, portMAX_DELAY);
    dust_ad_idx = 0;

    gpio_set_level(DUST_FAN_GPIO, 1); // FAN on
    vTaskDelay(10000 / portTICK_RATE_MS); // wait 10sec

    mcpwm_config_t pwm_config;
    pwm_config.frequency    = 100; // period is 10ms
    pwm_config.cmpr_a       = 0;
    pwm_config.cmpr_b       = 2.8; // sampling timing is 0.28ms later
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode    = MCPWM_DUTY_MODE_0;

    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config));
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0,
                                         MCPWM_OPR_A, 320));
    ESP_ERROR_CHECK(mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0));
}

static void dust_sense_stop()
{
    gpio_set_level(DUST_FAN_GPIO, 0); // FAN off
    ESP_ERROR_CHECK(mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
}

static int cmp_uint32(const uint32_t *a, const uint32_t *b)
{
    if (*a < *b) {
        return -1;
    } else if (*a == *b) {
        return 0;
    } else {
        return 1;
    }
}

static uint32_t dust_sense_ad_value()
{
    uint32_t ave = 0;

    // wait until the adc is done
    xSemaphoreTake(sense_done, portMAX_DELAY);
    xSemaphoreGive(sense_done);

    // extract the mean values, and average them
    qsort(dust_ad_list, ARRAY_SIZE(dust_ad_list), sizeof(uint32_t),
          (int (*)(const void *, const void *))cmp_uint32);
    for (uint32_t i = 0; i < DUST_AD_AVE_NUM; i++) {
        ave += dust_ad_list[ARRAY_SIZE(dust_ad_list)/2 - (DUST_AD_AVE_NUM/2) + i];
    }

    dust_ad_idx = 0;

    return ave / DUST_AD_AVE_NUM;
}

static uint32_t dust_get_vs(float temp)
{
    float temp_diff;

    temp_diff = temp - DUST_CALIB_TEMP;

    // 100mV per 15 celsius
    return DUST_CALIB_VOL + (temp_diff * 100 / 15);
}

static uint32_t dust_sense(float temp, float humi)
{
    uint32_t vo;
    uint32_t vs;
    int32_t v_diff;
    float alpha;
    float beta;

    vo = dust_sense_ad_value();
    ESP_LOGI(TAG, "PM2.5 VO = %d", vo);

    vs = dust_get_vs(temp);
    v_diff = vo - vs;
    if (v_diff < 0) {
        v_diff = 0;
    }

    alpha = 0.6;
    if (humi > 50) {
        beta = 1 - (0.01467 * (humi - 50));
    } else {
        beta = 1;
    }

    return (uint32_t)(alpha * beta * v_diff);
}

//////////////////////////////////////////////////////////////////////
// SHT3x (temperature and humidity sensor)
uint8_t crc8(const uint8_t *data, uint32_t len) {
    static const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool sht3x_sense_value_check_crc(uint16_t data, uint8_t crc) {
    uint8_t buf[2];

    buf[0] = (uint8_t)((data >> 8) & 0xFF);
    buf[1] = (uint8_t)((data >> 0) & 0xFF);

    return crc8(buf, 2) == crc;
}

static bool sht3x_check_crc(sht3x_sense_data_t *sense_data) {
    return sht3x_sense_value_check_crc(sense_data->temp, sense_data->temp_crc) &&
        sht3x_sense_value_check_crc(sense_data->humi, sense_data->humi_crc);
}

static float sht3x_calc_temp(sht3x_sense_data_t *sense_data) {
    return -45 + (175 * (sense_data->temp & 0xFFFF)) / (float)((1 << 16) - 1);
}

static float sht3x_calc_humi(sht3x_sense_data_t *sense_data) {
    return (100 * (sense_data->humi & 0xFFFF)) / (float)((1 << 16) - 1);
}

static esp_err_t sht3x_sense(sht3x_sense_value_t *sense_value)
{
    i2c_cmd_handle_t cmd;
    sht3x_sense_data_t sense_data;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SHT3x_DEV_ADDR|I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SHT3x_CMD_MSB, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SHT3x_CMD_LSB, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(24/portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, SHT3x_DEV_ADDR|I2C_MASTER_READ, 1));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.temp_bytes[1]), 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.temp_bytes[0]), 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.temp_crc), 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.humi_bytes[1]), 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.humi_bytes[0]), 0));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &(sense_data.humi_crc), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    if (sht3x_check_crc(&sense_data) == 0) {
        return ESP_FAIL;
    }

    sense_value->temp = sht3x_calc_temp(&sense_data);
    sense_value->humi = sht3x_calc_humi(&sense_data);

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// Sleep
void sleep_task(void *param)
{
   vTaskDelay(WAIT_BEFORE_SLEEP * 1000 / portTICK_RATE_MS);
   ESP_LOGI(TAG, "START SLEEPING...");

   esp_sleep_enable_timer_wakeup((SENSE_INTERVAL-WAIT_BEFORE_SLEEP) * 1000 * 1000);
   esp_deep_sleep_start();
}

//////////////////////////////////////////////////////////////////////
// WiFI
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, WIFI_HOSTNAME));
        ESP_ERROR_CHECK(esp_wifi_connect());

        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        wifi_status = wifi_connected;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        wifi_status = wifi_disconnected;
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void init_wifi()
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#ifdef WIFI_SSID
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    wifi_config_t wifi_config_cur;
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config_cur));

    if (strcmp((const char *)wifi_config_cur.sta.ssid, (const char *)wifi_config.sta.ssid) ||
        strcmp((const char *)wifi_config_cur.sta.password, (const char *)wifi_config.sta.password)) {
        ESP_LOGI(TAG, "SAVE WIFI CONFIG");
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
#endif
}

static void connect_wifi()
{
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void disconnect_wifi()
{
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_stop());
}

static int connect_server()
{
    struct sockaddr_in server;
    int sock;

    sock = socket(AF_INET, SOCK_STREAM, 0);

    server.sin_family = AF_INET;
    server.sin_port = htons(FLUENTD_PORT);
    server.sin_addr.s_addr = inet_addr(FLUENTD_IP);

    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
        ESP_LOGE(TAG, "FLUENTD CONNECT FAILED errno=%d", errno);
        return -1;
    }
    ESP_LOGI(TAG, "FLUENTD CONNECT SUCCESS");

    return sock;
}

static cJSON *sense_json(uint32_t pm25_value, sht3x_sense_value_t *sht3x_value,
                         wifi_ap_record_t *ap_record)
{
    cJSON *root = cJSON_CreateArray();
    cJSON *item = cJSON_CreateObject();

    cJSON_AddNumberToObject(item, "pm25", pm25_value);
    cJSON_AddNumberToObject(item, "temp", sht3x_value->temp);
    cJSON_AddNumberToObject(item, "humi", sht3x_value->humi);

    cJSON_AddStringToObject(item, "hostname", WIFI_HOSTNAME);
    cJSON_AddNumberToObject(item, "wifi_ch", ap_record->primary);
    cJSON_AddNumberToObject(item, "wifi_rssi", ap_record->rssi);

    cJSON_AddNumberToObject(item, "self_time", 0); // for Fluentd

    cJSON_AddItemToArray(root, item);

    return root;
}


static void process_sense_data(uint32_t time_start,
                               uint32_t pm25_value, sht3x_sense_value_t *sht3x_value)
{
    wifi_ap_record_t ap_record;
    char buffer[sizeof(EXPECTED_RESPONSE)];

    while (wifi_status != wifi_connected) {
        if ((xTaskGetTickCount() - time_start)> (WIFI_CONNECT_TIMEOUT * 1000 / portTICK_PERIOD_MS)) {
            ESP_LOGE(TAG, "WIFI CONNECT TIMECOUT");
            return;
        }
        vTaskDelay(100 / portTICK_RATE_MS); // wait 100ms
    }
    ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_record));

    int sock = connect_server();
    if (sock == -1) {
        return;
    }

    cJSON *json = sense_json(pm25_value, sht3x_value, &ap_record);
    char *json_str = cJSON_PrintUnformatted(json);

    do {
        if (dprintf(sock, REQUEST, strlen("json=") + strlen(json_str), json_str) < 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }

        bzero(buffer, sizeof(buffer));
        read(sock, buffer, sizeof(buffer)-1);

        if (strcmp(buffer, EXPECTED_RESPONSE) != 0) {
            ESP_LOGE(TAG, "FLUENTD POST FAILED");
            break;
        }
        ESP_LOGI(TAG, "FLUENTD POST SUCCESSFUL");
    } while (0);

    close(sock);
    cJSON_Delete(json);
}

//////////////////////////////////////////////////////////////////////
static void init_gpio()
{
    gpio_config_t io_conf;
    i2c_config_t conf;

    // GPIO
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << DUST_FAN_GPIO;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(DUST_FAN_GPIO, 0);

    // PWM
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, DUST_LED_GPIO));

    MCPWM[MCPWM_UNIT_0]->int_ena.val = 1 << 18;
    ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, isr_handler,
                                       NULL, ESP_INTR_FLAG_IRAM, NULL));

    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DUST_ADC_CH, ADC_ATTEN_11db);
    esp_adc_cal_get_characteristics(
        ADC_VREF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
        &adc_char
    );

    // I2C
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void app_main()
{
    sht3x_sense_value_t sht3x_sense_value = { 0, 0 };
    uint32_t pm25_value;
    esp_err_t sht3x_sense_err;
    uint32_t time_start;

    xTaskCreate(&sleep_task, "sleep_task", 2048, NULL, 1, NULL);

    init_gpio();
    init_wifi();

    vSemaphoreCreateBinary(sense_done);

    sht3x_sense_err = sht3x_sense(&sht3x_sense_value);

    if (sht3x_sense_err == ESP_OK) {
        dust_sense_start();
        pm25_value = dust_sense(sht3x_sense_value.temp, sht3x_sense_value.humi);
        dust_sense_stop();

        time_start = xTaskGetTickCount();
        connect_wifi();
        process_sense_data(time_start, pm25_value, &sht3x_sense_value);
        disconnect_wifi();
    }

    ESP_LOGI(TAG, "FREE HEAP SIZE: %d", esp_get_free_heap_size());
    ESP_LOGI(TAG, "MAX STACK SIZE: %d", uxTaskGetStackHighWaterMark(NULL));
}
