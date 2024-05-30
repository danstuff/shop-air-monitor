#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "ssd1366.h"

#define log(...) ESP_LOGI("sam", __VA_ARGS__)

#define MHz_to_Hz(x) ((x) * 1000 * 1000)

typedef enum _status_code_t
{
    STATUS_CODE_OK,
    STATUS_CODE_NO_IN,
    STATUS_CODE_NO_OUT,
    STATUS_CODE_NO_IO,
} status_code_t;

static const uint8_t STATUS_COLORS[][3] =
{
    [STATUS_CODE_OK] = { 0, 255, 0 },
    [STATUS_CODE_NO_IN] = { 255, 127, 0 },
    [STATUS_CODE_NO_OUT] = { 255, 240, 0 },
    [STATUS_CODE_NO_IO] = { 255, 0, 0 },
};

typedef enum _particle_t
{
    PARTICLE_TYPE_0P3,
    PARTICLE_TYPE_0P5,
    PARTICLE_TYPE_1P0,
    PARTICLE_TYPE_2P5,
    PARTICLE_TYPE_5P0,
    PARTICLE_TYPE_50,
    PARTICLE_TYPE_COUNT,
} particle_t;

typedef struct _lstate_t
{
    uint8_t led_is_on;
    led_strip_handle_t led_handle;
    
    uint8_t kparticles[PARTICLE_TYPE_COUNT];
} lstate_t;

static lstate_t lstate = { 0 };

//#define ENABLE_OLED

#ifdef ENABLE_OLED
static const uint8_t oled_bkg[] =
{
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,67,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,66,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,66,64,0,0,0,0,0,0,0,0,0,0,0,0,0,224,83,128,0,0,0,0,0,0,0,0,0,0,0,0,1,240,98,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,82,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,74,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,8,0,0,0,0,0,0,0,0,0,0,0,0,0,1,240,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,64,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,224,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0,0,0,9,20,8,0,0,0,0,0,0,0,0,0,0,0,0,0,9,42,8,0,0,0,0,0,0,0,0,0,0,0,0,0,9,42,15,255,255,255,255,255,255,255,255,255,255,255,255,255,201,34,0,0,0,0,0,0,0,0,0,0,0,0,0,0,14,162,0,195,128,24,120,6,6,0,225,224,30,24,3,204,8,0,1,32,64,36,64,2,9,0,17,0,16,36,2,18,8,0,1,33,128,36,112,2,9,0,17,192,28,36,3,146,0,0,1,32,64,36,8,2,9,0,96,32,2,36,0,82,0,0,1,32,64,36,8,2,9,0,128,32,2,36,0,82,0,0,0,203,128,25,112,7,38,0,245,192,28,152,3,140,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
#endif

static uint8_t oled_write(uint8_t* data, size_t data_len)
{
    /* Open a command link and write given data to it */
    i2c_cmd_handle_t i2c_cmd = i2c_cmd_link_create();

    i2c_master_start(i2c_cmd);
    i2c_master_write(i2c_cmd, data, data_len, true);
    i2c_master_stop(i2c_cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(
            I2C_NUM_0, i2c_cmd, CONFIG_OLED_WAIT_MS / portTICK_PERIOD_MS));

    i2c_cmd_link_delete(i2c_cmd);
    return 0;
}

static void update(void)
{
    status_code_t status = STATUS_CODE_OK;
    uint8_t in_packet[32] = { 0 };

    int in_packet_len = uart_read_bytes(
            CONFIG_AQS_UART_PORT, 
            in_packet,
            sizeof(in_packet),
            CONFIG_AQS_WAIT_MS / portTICK_PERIOD_MS);

    if (in_packet_len > 0)
    {
        log("Got AQS data");
        for (int i = 0; i < in_packet_len; i++)
        {
            log("%x", in_packet[i]);
        }
        /* TODO convert AQS data and put it in lstate.kparticles[] */
    }
    else
    {
        log("[WARN] Got no AQS data %i", in_packet_len);
        status = STATUS_CODE_NO_IN;
    }

#ifdef ENABLE_OLED
    /* Print sensor data to screen */
    uint8_t out_packet[4 + 128] =
    {
        (CONFIG_OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        OLED_CONTROL_BYTE_CMD_SINGLE,
        0xB0,
        OLED_CONTROL_BYTE_DATA_STREAM,
    };
    for (uint8_t page = 0; page < 8; page++)
    {
        out_packet[2] = 0xB0 | page;
        for (uint8_t byte = 0; byte < 128; byte++)
        {
            /* Write out the background image */
            out_packet[4 + byte] |= oled_bkg[(page * 128) + byte];
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                /* TODO check if a column should be drawn here based on lstate.kparticles */
            }
        }
        if (oled_write(out_packet, sizeof(out_packet)) != ESP_OK)
        {
            log("[WARN] Could not write to OLED page %u", page);
            status = (status == STATUS_CODE_NO_IN) ? STATUS_CODE_NO_IO : STATUS_CODE_NO_OUT;
        }
    }
#endif
    /* Update status LED */
    led_strip_set_pixel(lstate.led_handle,
            0,
            STATUS_COLORS[status][0],
            STATUS_COLORS[status][1],
            STATUS_COLORS[status][2]);
    led_strip_refresh(lstate.led_handle);
}

static void startup(void)
{
    log("Starting up...");

    /* Get a handle for the board's status LED */
    led_strip_config_t strip_config = 
    {
        .strip_gpio_num = CONFIG_STATUS_LED_DATA_PIN,
        .max_leds = 1, 
    };
    led_strip_rmt_config_t rmt_config = 
    {
        .resolution_hz = MHz_to_Hz(CONFIG_STATUS_LED_RESOLUTION_MHZ),
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &lstate.led_handle));

    /* Start with LED off */
    led_strip_clear(lstate.led_handle);

    /* Enable the LED power pin */
    gpio_reset_pin(CONFIG_STATUS_LED_POWER_PIN);
    gpio_set_direction(CONFIG_STATUS_LED_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_STATUS_LED_POWER_PIN, 1);

    /* Open a port to the UART air quality sensor */
    uart_config_t uart_config =
    {
        .baud_rate = CONFIG_AQS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(
                CONFIG_AQS_UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(
                CONFIG_AQS_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(
                CONFIG_AQS_UART_PORT, 0, CONFIG_UART_RX_PIN, 0, 0));

#ifdef ENABLE_OLED
    /* Open I2C connection to the ssd1306 display */
    /* Based on https://github.com/yanbe/ssd1306-esp-idf-i2c/ */
    i2c_config_t i2c_config = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_SDA_PIN,
        .scl_io_num = CONFIG_I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MHz_to_Hz(CONFIG_OLED_CLOCK_SPEED),
    };
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));

    /* Configure and turn on the display */
    uint8_t oled_init_msg[] =
    {
        (CONFIG_OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
        OLED_CONTROL_BYTE_CMD_STREAM,
        OLED_CMD_SET_CHARGE_PUMP,
        0x14,
        OLED_CMD_SET_SEGMENT_REMAP, /* Mirror display X axis */
        OLED_CMD_SET_COM_SCAN_MODE, /* Mirror display Y axis */
        OLED_CMD_DISPLAY_ON,
    };
    ESP_ERROR_CHECK(oled_write(oled_init_msg, sizeof(oled_init_msg)));
#endif
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    startup();

    /* Enter infinite loop blinking LED, update when LED turns on */
    while (1) { 
        if (lstate.led_is_on) {
            update();
        } else {
            led_strip_clear(lstate.led_handle);
        }
        lstate.led_is_on = !lstate.led_is_on;
        vTaskDelay(CONFIG_UPDATE_MS / (portTICK_PERIOD_MS * 2));
    }
}
