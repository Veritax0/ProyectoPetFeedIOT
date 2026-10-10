#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "SISTEMA";

#define RFID_UART_PORT      UART_NUM_2
#define RFID_RX_PIN         16
#define RFID_TX_PIN         UART_PIN_NO_CHANGE
#define RFID_BAUD_RATE      9600
#define RFID_BUF_SIZE       256

#define RFID_STX             0x02
#define RFID_ETX             0x03
#define RFID_PACKET_LEN       14

#define RFID_QUEUE_LENGTH 10

typedef struct {
    uint32_t tagId;
    char     tagHex[11];
} rfid_message_t;

QueueHandle_t rfidQueue;

#define HX711_DOUT_PIN  18
#define HX711_SCK_PIN   19

#define SERVO_PIN           23
#define SERVO_LEDC_TIMER    LEDC_TIMER_0
#define SERVO_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL  LEDC_CHANNEL_0
#define SERVO_LEDC_FREQ_HZ  50
#define SERVO_LEDC_RES      LEDC_TIMER_14_BIT

#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500

static uint8_t hexCharToValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

static uint8_t hexPairToByte(char high, char low)
{
    return (hexCharToValue(high) << 4) | hexCharToValue(low);
}

static bool rfidValidarChecksum(const uint8_t *data)
{
    uint8_t bytes[5];
    for (int i = 0; i < 5; i++) {
        bytes[i] = hexPairToByte(data[1 + i * 2], data[2 + i * 2]);
    }

    uint8_t checksumCalculado = 0;
    for (int i = 0; i < 5; i++) {
        checksumCalculado ^= bytes[i];
    }

    uint8_t checksumRecibido = hexPairToByte(data[11], data[12]);
    return checksumCalculado == checksumRecibido;
}

static void rfidUartInit(void)
{
    uart_config_t uartConfig = {
        .baud_rate  = RFID_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RFID_UART_PORT, RFID_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RFID_UART_PORT, &uartConfig));
    ESP_ERROR_CHECK(uart_set_pin(RFID_UART_PORT, RFID_TX_PIN, RFID_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void hx711Init(void)
{
    gpio_config_t doutConfig = {
        .pin_bit_mask = (1ULL << HX711_DOUT_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&doutConfig);

    gpio_config_t sckConfig = {
        .pin_bit_mask = (1ULL << HX711_SCK_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&sckConfig);

    gpio_set_level(HX711_SCK_PIN, 0);
}

static bool hx711IsReady(void)
{
    return gpio_get_level(HX711_DOUT_PIN) == 0;
}

static int32_t hx711ReadRaw(void)
{
    int timeoutCounter = 0;
    while (!hx711IsReady()) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (++timeoutCounter > 500) {
            ESP_LOGW(TAG, "HX711 timeout");
            return 0;
        }
    }

    int32_t value = 0;

    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        value = (value << 1) | gpio_get_level(HX711_DOUT_PIN);
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);

    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    return value;
}

static void servoInit(void)
{
    ledc_timer_config_t timerConfig = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES,
        .freq_hz         = SERVO_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    ledc_channel_config_t channelConfig = {
        .gpio_num   = SERVO_PIN,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
}

static void servoSetAngle(int angle)
{
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    int pulseUs = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;

    uint32_t maxDuty = (1 << SERVO_LEDC_RES) - 1;
    uint32_t periodUs = 1000000 / SERVO_LEDC_FREQ_HZ;
    uint32_t duty = (uint32_t)(((uint64_t)pulseUs * maxDuty) / periodUs);

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

void rfidReaderTask(void *pvParameters)
{
    uint8_t rxByte;
    uint8_t buffer[RFID_PACKET_LEN];
    uint8_t bufferIndex = 0;
    bool receiving = false;

    while (1)
    {
        int len = uart_read_bytes(RFID_UART_PORT, &rxByte, 1, pdMS_TO_TICKS(20));
        if (len <= 0) continue;
        if (rxByte == RFID_STX) {
            receiving = true;
            bufferIndex = 0;
            buffer[bufferIndex++] = rxByte;
        } else if (receiving) {
            buffer[bufferIndex++] = rxByte;

            if (bufferIndex >= RFID_PACKET_LEN) {
                receiving = false;

                if (buffer[RFID_PACKET_LEN - 1] == RFID_ETX &&
                    rfidValidarChecksum(buffer))
                {
                    rfid_message_t msg;
                    memcpy(msg.tagHex, &buffer[1], 10);
                    msg.tagHex[10] = '\0';
                    msg.tagId = (uint32_t)strtoul(&msg.tagHex[2], NULL, 16);

                    xQueueSend(rfidQueue, &msg, pdMS_TO_TICKS(100));
                } else {
                    ESP_LOGW(TAG, "RFID paquete invalido");
                }
                bufferIndex = 0;
            }
        }
    }
}

void rfidConsumerTask(void *pvParameters)
{
    rfid_message_t receivedMsg;

    while (1)
    {
        if (xQueueReceive(rfidQueue, &receivedMsg, portMAX_DELAY) == pdPASS)
        {
            ESP_LOGI(TAG, "RFID tag HEX=%s decimal=%lu",
                     receivedMsg.tagHex, (unsigned long)receivedMsg.tagId);
        }
    }
}

void loadCellTask(void *pvParameters)
{
    while (1)
    {
        int32_t rawValue = hx711ReadRaw();
        ESP_LOGI(TAG, "HX711 valor crudo: %" PRId32, rawValue);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void servoTask(void *pvParameters)
{
    int angles[] = {0, 90, 180, 90};
    int idx = 0;

    while (1)
    {
        int angle = angles[idx];
        servoSetAngle(angle);
        ESP_LOGI(TAG, "Servo posicion: %d", angle);

        idx = (idx + 1) % (sizeof(angles) / sizeof(angles[0]));
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    rfidUartInit();
    hx711Init();
    servoInit();

    rfidQueue = xQueueCreate(RFID_QUEUE_LENGTH, sizeof(rfid_message_t));
    if (rfidQueue == NULL) {
        ESP_LOGE(TAG, "Error creando rfidQueue");
        return;
    }

    xTaskCreate(rfidReaderTask,   "RFID_Reader",   4096, NULL, 2, NULL);
    xTaskCreate(rfidConsumerTask, "RFID_Consumer", 4096, NULL, 1, NULL);
    xTaskCreate(loadCellTask,     "LoadCell",      4096, NULL, 1, NULL);
    xTaskCreate(servoTask,        "Servo",         4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "Sistema inicializado");
}