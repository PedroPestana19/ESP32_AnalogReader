#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "esp_adc_cal.h"

/*---------------------------------DEFINES---------------------------*/

/*----EXTRA--------*/
#define TEST printf("---TESTE---")

/*----I2S----------*/
#define FINAL_FREQ 1000
#define FREQUENCY (FINAL_FREQ * 2)
#define MULTI_SAMPLING 15
#define SAMPLING_FREQ (FREQUENCY * MULTI_SAMPLING)
#define I2S_PORT I2S_NUM_0
#define SAMPLE_MASK 0Xfff
#define I2S_EVENT_QUEUE_ITEM_COUNT 10
#define DMA_BUF_COUNT 4

/*----INDICATORS----*/
#define FREQ_PIN GPIO_NUM_14
#define LED_PIN GPIO_NUM_22
#define LED_FREQ 500

/*----ADC-----------*/
#define V_REF 1100

/*---------------------------------VARIABLES-------------------------*/
QueueHandle_t I2S_Event_Queue = NULL;
QueueHandle_t Data_Queue = NULL;
uint16_t samples[MULTI_SAMPLING];
bool pin = false;
bool led = false;

const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = MULTI_SAMPLING,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

esp_adc_cal_characteristics_t adc_config;

/*---------------------------------FUNCTIONS-------------------------*/
static void GPIO_Config();
static void I2S_Reader(void *pvParameters);
static void Data_Transmiter(void *pvParameters);
static void I2S_Config(void);
static int sort_desc(const void *a, const void *b);
static void esp_error(const char *cause, esp_err_t err);
static void printSamples(void);
static uint16_t *sampleFilter(uint16_t *buff, uint8_t size);

/*---------------------------------SETUP-----------------------------*/
void app_main()
{
    GPIO_Config();

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &adc_config);

    I2S_Event_Queue = xQueueCreate(I2S_EVENT_QUEUE_ITEM_COUNT, sizeof(i2s_event_t));
    if (I2S_Event_Queue == NULL)
        esp_error("I2S Event queue not initializes!!", -1);

    Data_Queue = xQueueCreate(10, sizeof(uint16_t));
    if (Data_Queue == NULL)
        esp_error("Data queue not initializes!!", -1);

    I2S_Config();
    if (xTaskCreate(I2S_Reader, "I2S_TASK", 1024 * 2, NULL, 1, NULL) != pdPASS)
        esp_error("I2S_Reader not created!!", -1);

    if (xTaskCreate(Data_Transmiter, "Date_Processor", 1024 * 2, NULL, 1, NULL) != pdPASS)
        esp_error("Data_Transmiter not created!!", -1);
}

/*---------------------------------MY TASKS--------------------------*/
void I2S_Reader(void *pvParameters)
{
    configASSERT(pvParameters == NULL);
    i2s_event_t evt;
    while (pdTRUE)
    {
        if (xQueueReceive(I2S_Event_Queue, &evt, portMAX_DELAY) == pdPASS)
        {
            if (evt.type == I2S_EVENT_RX_DONE)
            {
                size_t bytesRead = 0;
                do
                {
                    i2s_read(I2S_PORT, samples + bytesRead, MULTI_SAMPLING * 2, &bytesRead, portMAX_DELAY);
                } while (bytesRead < MULTI_SAMPLING);
                
                if (!xQueueSend(Data_Queue, sampleFilter(samples, MULTI_SAMPLING), pdMS_TO_TICKS(2)))
                    esp_error("Send error", -1);

                gpio_set_level(FREQ_PIN, pin ^= pdTRUE);
            }
        }
    }
}

void Data_Transmiter(void *pvParameters)
{
    configASSERT(pvParameters == NULL);
    uint16_t sample = 0;
    int ledCount = 0;
    while (pdTRUE)
    {
        if (xQueueReceive(Data_Queue, &sample, portMAX_DELAY) == pdTRUE)
        {
            sample &= SAMPLE_MASK;
            printf("%d\n", esp_adc_cal_raw_to_voltage( sample, &adc_config));
            if (ledCount >= LED_FREQ)
            {
                gpio_set_level(LED_PIN, led ^= pdTRUE);
                ledCount = 0;
            }
            ledCount++;
        }
    }
}

/*---------------------------------MY FUNCS--------------------------*/

static void GPIO_Config(void){
    gpio_pad_select_gpio(FREQ_PIN);
    gpio_set_direction(FREQ_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FREQ_PIN, pdFALSE);

    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, pdFALSE);
}

static void esp_error(const char *cause, esp_err_t err)
{
    if (err != ESP_OK)
    {
        printf("Failed to %s: %d\n", cause, err);
        while (true)
            ;
    }
}

static uint16_t *sampleFilter(uint16_t *buff, uint8_t size)
{
    qsort(buff, size, sizeof(uint16_t), sort_desc);
    return buff + (size / 2);
}

static void printSamples(void)
{
    for (int i = 0; i < MULTI_SAMPLING; ++i)
    {
        printf("%x ", samples[i]);
    }
    printf("\n");
}

static int sort_desc(const void *a, const void *b)
{
    return *((uint16_t *)b) - *((uint16_t *)a);
}

static void I2S_Config(void)
{
    printf("Sample Rate = %d\nBits per Sample = %d\nNumber DMA Buffers = %d\nSize DMA per Buffer = %d\n",
           SAMPLING_FREQ, I2S_BITS_PER_SAMPLE_16BIT, DMA_BUF_COUNT, MULTI_SAMPLING);
    esp_err_t err;
    // The I2S config as per the example

    err = adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_7);
    esp_error("setup adc channel", err);

    err = i2s_driver_install(I2S_NUM_0, &i2s_config, I2S_EVENT_QUEUE_ITEM_COUNT, &I2S_Event_Queue);
    esp_error("inicialize driver", err);

    err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_7);
    esp_error("setup adc mode", err);
}