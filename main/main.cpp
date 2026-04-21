/*
    name: Esp32 serial flasher
    from: M.Mastenbroek (github.com/Machiel80)
*/

#include <sys/param.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "esp_err.h"
// #define LOG_LOCAL_LEVEL ESP_LOG_INFO // ESP_LOG_DEBUG, ESP_LOG_VERBOSE
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

#ifdef DISPLAY_ENABLED
#include "driver/i2c_master.h"
#endif

#include "esp_loader.h"

extern "C"
{
#include "esp32_port.h"
#include "firmware_target.h"
#include "button.h"
}

volatile bool _toggle_validate_slave_software_task_enabled = true;
volatile bool _toggle_result_validate_slave_software_changed = true;
static SemaphoreHandle_t _button_semaphore = NULL;
static volatile bool _button_pressed = false;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;
    _button_pressed = true;
    if (_button_semaphore != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(_button_semaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
    }
}

#ifdef DISPLAY_ENABLED
static bool _lcd_ready = false;
static uint8_t _lcd_i2c_addr = 0; // Discovered I2C address
static i2c_master_bus_handle_t _lcd_i2c_bus = NULL;
static i2c_master_dev_handle_t _lcd_i2c_dev = NULL;

static const uint8_t LCD_PIN_RS = (1 << LCD_PIN_RS_BIT);
static const uint8_t LCD_PIN_RW = (1 << LCD_PIN_RW_BIT);
static const uint8_t LCD_PIN_EN = (1 << LCD_PIN_EN_BIT);
static const uint8_t LCD_PIN_BL = (1 << LCD_PIN_BL_BIT);
static const uint32_t LCD_ENABLE_PULSE_US = 2;
static const uint32_t LCD_ENABLE_SETTLE_US = 100;
static const uint32_t LCD_CLEAR_DELAY_US = 3000;
static const uint32_t LCD_BOOT_SCREEN_DURATION_MS = 5000;
static const uint32_t LCD_MARQUEE_INTERVAL_MS = 450;
static const uint8_t LCD_MARQUEE_TAIL_SPACES = 4;

static bool _lcd_validate_marquee_enabled = false;
static char _lcd_validate_marquee_text[48] = {0};
static size_t _lcd_validate_marquee_offset = 0;
static int64_t _lcd_validate_last_step_ms = 0;

static esp_err_t lcd_i2c_write(uint8_t data)
{
    if (_lcd_i2c_dev == NULL)
    {
        return ESP_FAIL;
    }
    return i2c_master_transmit(_lcd_i2c_dev, &data, 1, 50);
}

static esp_err_t lcd_check_err(esp_err_t err, const char *msg)
{
    if (err != ESP_OK)
    {
        ESP_LOGE("lcd", "%s (err=%d)", msg, (int)err);
    }
    return err;
}

static esp_err_t lcd_write_expander(uint8_t data)
{
    esp_err_t err = lcd_i2c_write(data);
    if (err == ESP_OK)
    {
        esp_rom_delay_us(LCD_ENABLE_SETTLE_US);
    }
    return err;
}

static esp_err_t lcd_write4(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (uint8_t)((nibble << LCD_DATA_SHIFT) | LCD_PIN_BL | (rs ? LCD_PIN_RS : 0));
    esp_err_t err = lcd_write_expander(data);
    if (err != ESP_OK)
    {
        return err;
    }

    err = lcd_write_expander((uint8_t)(data | LCD_PIN_EN));
    if (err != ESP_OK)
    {
        return err;
    }

    esp_rom_delay_us(LCD_ENABLE_PULSE_US);

    err = lcd_write_expander(data);
    if (err != ESP_OK)
    {
        return err;
    }

    esp_rom_delay_us(LCD_ENABLE_SETTLE_US);
    return ESP_OK;
}

static esp_err_t lcd_send(uint8_t value, uint8_t rs)
{
    esp_err_t err = lcd_write4((uint8_t)(value >> 4), rs);
    if (err != ESP_OK)
    {
        return err;
    }

    err = lcd_write4((uint8_t)(value & 0x0F), rs);
    if (err != ESP_OK)
    {
        return err;
    }

    if (!rs && (value == 0x01 || value == 0x02))
    {
        esp_rom_delay_us(LCD_CLEAR_DELAY_US);
    }
    else
    {
        esp_rom_delay_us(LCD_ENABLE_SETTLE_US);
    }

    return ESP_OK;
}

static esp_err_t lcd_cmd(uint8_t cmd)
{
    return lcd_send(cmd, 0);
}

static esp_err_t lcd_data(uint8_t data)
{
    return lcd_send(data, 1);
}

static esp_err_t lcd_set_cursor(uint8_t row, uint8_t col)
{
    static const uint8_t row_addr[] = {0x00, 0x40};
    if (row > 1 || col > 15)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return lcd_cmd((uint8_t)(0x80 | (row_addr[row] + col)));
}

static esp_err_t lcd_print_line(uint8_t row, const char *text)
{
    static const uint32_t LCD_SCROLL_DELAY_MS = 250;
    static const uint8_t LCD_SCROLL_TAIL_SPACES = 4;

    if (text == NULL)
    {
        text = "";
    }

    size_t len = strlen(text);
    esp_err_t err = ESP_OK;

    if (len <= 16)
    {
        err = lcd_set_cursor(row, 0);
        if (err != ESP_OK)
        {
            return err;
        }

        for (int i = 0; i < 16; ++i)
        {
            char c = ' ';
            if (text[i] != '\0')
            {
                c = text[i];
            }
            err = lcd_data((uint8_t)c);
            if (err != ESP_OK)
            {
                return err;
            }
        }
        return ESP_OK;
    }

    for (size_t offset = 0; offset < (len + LCD_SCROLL_TAIL_SPACES); ++offset)
    {
        err = lcd_set_cursor(row, 0);
        if (err != ESP_OK)
        {
            return err;
        }

        for (int col = 0; col < 16; ++col)
        {
            size_t src = offset + (size_t)col;
            char c = ' ';
            if (src < len)
            {
                c = text[src];
            }

            err = lcd_data((uint8_t)c);
            if (err != ESP_OK)
            {
                return err;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LCD_SCROLL_DELAY_MS));
    }

    return ESP_OK;
}

static esp_err_t lcd_print_centered_line(uint8_t row, const char *text)
{
    if (text == NULL)
    {
        text = "";
    }

    size_t len = strlen(text);
    if (len >= 16)
    {
        return lcd_print_line(row, text);
    }

    char line[17] = {0};
    memset(line, ' ', 16);
    size_t left = (16 - len) / 2;
    memcpy(&line[left], text, len);
    line[16] = '\0';
    return lcd_print_line(row, line);
}

static esp_err_t lcd_render_scroll_frame(uint8_t row, const char *text, size_t offset)
{
    if (text == NULL)
    {
        text = "";
    }

    size_t len = strlen(text);
    if (len <= 16)
    {
        return lcd_print_line(row, text);
    }

    esp_err_t err = lcd_set_cursor(row, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    for (int col = 0; col < 16; ++col)
    {
        size_t src = offset + (size_t)col;
        char c = ' ';
        if (src < len)
        {
            c = text[src];
        }
        err = lcd_data((uint8_t)c);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}

static void lcd_update_validated_marquee(void)
{
    if (!_lcd_ready || !_lcd_validate_marquee_enabled)
    {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000LL;
    if (_lcd_validate_last_step_ms != 0 && (now_ms - _lcd_validate_last_step_ms) < LCD_MARQUEE_INTERVAL_MS)
    {
        return;
    }

    size_t len = strlen(_lcd_validate_marquee_text);
    if (len == 0)
    {
        return;
    }

    if (lcd_render_scroll_frame(1, _lcd_validate_marquee_text, _lcd_validate_marquee_offset) == ESP_OK)
    {
        size_t cycle = len + LCD_MARQUEE_TAIL_SPACES;
        _lcd_validate_marquee_offset = (_lcd_validate_marquee_offset + 1) % cycle;
        _lcd_validate_last_step_ms = now_ms;
    }
}

static void lcd_show_boot_text(void)
{
    static const char *BOOT_ROW2 = "Starting... Please wait";

    if (!_lcd_ready)
    {
        return;
    }

    lcd_print_centered_line(0, "FlashBox Boot");

    size_t len = strlen(BOOT_ROW2);
    if (len <= 16)
    {
        lcd_print_line(1, BOOT_ROW2);
        vTaskDelay(pdMS_TO_TICKS(LCD_BOOT_SCREEN_DURATION_MS));
        return;
    }

    size_t cycle = len + LCD_MARQUEE_TAIL_SPACES;
    size_t offset = 0;
    int64_t start_ms = esp_timer_get_time() / 1000LL;

    while ((esp_timer_get_time() / 1000LL) - start_ms < LCD_BOOT_SCREEN_DURATION_MS)
    {
        lcd_render_scroll_frame(1, BOOT_ROW2, offset);
        offset = (offset + 1) % cycle;
        vTaskDelay(pdMS_TO_TICKS(LCD_MARQUEE_INTERVAL_MS));
    }
}

static esp_err_t lcd_init(void)
{
    esp_err_t err = ESP_OK;
    if (_lcd_i2c_bus == NULL)
    {
        i2c_master_bus_config_t bus_conf = {};
        bus_conf.i2c_port = LCD_I2C_PORT;
        bus_conf.sda_io_num = (gpio_num_t)LCD_I2C_SDA;
        bus_conf.scl_io_num = (gpio_num_t)LCD_I2C_SCL;
        bus_conf.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_conf.glitch_ignore_cnt = 7;
        bus_conf.intr_priority = 0;
        bus_conf.trans_queue_depth = 0;
        bus_conf.flags.enable_internal_pullup = 1;
        bus_conf.flags.allow_pd = 0;

        err = i2c_new_master_bus(&bus_conf, &_lcd_i2c_bus);
        if (err == ESP_ERR_INVALID_STATE)
        {
            err = i2c_master_get_bus_handle(LCD_I2C_PORT, &_lcd_i2c_bus);
        }
        if (err != ESP_OK)
        {
            return err;
        }
    }

    // Try to find LCD at common addresses
    const uint8_t addresses[] = {0x27, 0x3F, 0x3C, 0x38};
    uint8_t found_addr = 0;
    for (int i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++)
    {
        err = i2c_master_probe(_lcd_i2c_bus, addresses[i], 50);
        if (err == ESP_OK)
        {
            found_addr = addresses[i];
            ESP_LOGI("lcd", "LCD found at address 0x%02X", found_addr);
            break;
        }
    }

    if (found_addr == 0)
    {
        ESP_LOGE("lcd", "LCD not found at any address. Check connections and power.");
        return ESP_FAIL;
    }

    _lcd_i2c_addr = found_addr;

    i2c_device_config_t dev_conf = {};
    dev_conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_conf.device_address = _lcd_i2c_addr;
    dev_conf.scl_speed_hz = LCD_I2C_FREQ_HZ;
    dev_conf.scl_wait_us = 0;
    dev_conf.flags.disable_ack_check = 0;

    if (_lcd_i2c_dev != NULL)
    {
        i2c_master_bus_rm_device(_lcd_i2c_dev);
        _lcd_i2c_dev = NULL;
    }
    err = i2c_master_bus_add_device(_lcd_i2c_bus, &dev_conf, &_lcd_i2c_dev);
    if (err != ESP_OK)
    {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    err = lcd_write_expander(LCD_PIN_BL);
    if (err != ESP_OK)
    {
        return err;
    }
    err = lcd_check_err(lcd_write4(0x03, 0), "init 8-bit step 1 failed");
    if (err != ESP_OK)
    {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    err = lcd_check_err(lcd_write4(0x03, 0), "init 8-bit step 2 failed");
    if (err != ESP_OK)
    {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    err = lcd_check_err(lcd_write4(0x03, 0), "init 8-bit step 3 failed");
    if (err != ESP_OK)
    {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    err = lcd_check_err(lcd_write4(0x02, 0), "switch to 4-bit mode failed");
    if (err != ESP_OK)
    {
        return err;
    }

    err = lcd_check_err(lcd_cmd(0x28), "function set failed");
    if (err != ESP_OK)
    {
        return err;
    }
    err = lcd_check_err(lcd_cmd(0x08), "display off failed");
    if (err != ESP_OK)
    {
        return err;
    }
    err = lcd_check_err(lcd_cmd(0x01), "clear failed");
    if (err != ESP_OK)
    {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    err = lcd_check_err(lcd_cmd(0x06), "entry mode failed");
    if (err != ESP_OK)
    {
        return err;
    }
    err = lcd_check_err(lcd_cmd(0x0C), "display on failed");
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}
#endif
enum cookerFirmwareVersion _firmware_version = cfvUNKNOWN;

static void change_request_validate_software(const char *note, enum cookerFirmwareVersion firmware_version_state)
{
    static const char *TAG = "software changed";

    ESP_LOGD(TAG, "request '%s'", note);

    if (firmware_version_state != _firmware_version)
    {
        _firmware_version = firmware_version_state;
        _toggle_result_validate_slave_software_changed = true;
        if (_firmware_version == cfvVALIDATED)
            ESP_LOGD(TAG, "state changed '%s' to VALIDATED", note);
        else if (_firmware_version == cfvAMISS)
            ESP_LOGD(TAG, "state changed '%s' to AMISS", note);
        else
            ESP_LOGD(TAG, "state changed '%s' to UNKNOWN", note);
    }
}

static void slave_software_validate_task(void *arg)
{
    static const char *TAG = "validate software";
    static bool task_restart = false;

    unsigned long REQUEST_FOR_VERSION_INTERVAL = 1000; // ms example: 1000 = 1 sec.
    unsigned long RESPONSE_FOR_VERSION_TIMEOUT = 200;  // ms example: 1000 = 1 sec.
    unsigned long lastVersionRequestTime = 0;
    unsigned long lastVersionResponseTime = 0;

    bool version_request_sent = false;
    bool timeout_timer_active = false;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {};
    uart_config.baud_rate = VALIDATE_UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_APB;
    uart_config.flags.allow_pd = 0;

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(VALIDATE_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(VALIDATE_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(VALIDATE_UART_PORT_NUM, VALIDATE_TEST_TXD, VALIDATE_TEST_RXD, VALIDATE_TEST_RTS, VALIDATE_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    ESP_LOGI(TAG, "(I) task started");
    ESP_LOGD(TAG, "(D) task started");

    while (true)
    {
        if (_toggle_validate_slave_software_task_enabled == true)
        {
            if (task_restart == true)
            {
                task_restart = false;
                ESP_LOGD(TAG, "task restarted");
                ESP_ERROR_CHECK(uart_driver_install(VALIDATE_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
                ESP_ERROR_CHECK(uart_param_config(VALIDATE_UART_PORT_NUM, &uart_config));
                ESP_ERROR_CHECK(uart_set_pin(VALIDATE_UART_PORT_NUM, VALIDATE_TEST_TXD, VALIDATE_TEST_RXD, VALIDATE_TEST_RTS, VALIDATE_TEST_CTS));

                ESP_LOGI(TAG, "reset slave");
                esp_loader_reset_target();
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }

            // Read data from the UART
            int len = uart_read_bytes(VALIDATE_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

            if (len > 0)
            {
                if (version_request_sent == true)
                {
                    version_request_sent = false;
                    if (strstr((char *)data, TARGET_SOFTWARE_VERSION) != NULL)
                    {
                        change_request_validate_software("equal versions", cfvVALIDATED);
                        ESP_LOGD(TAG, "VALID (%.*s)\n", len, data);
                    }
                    else
                    {
                        change_request_validate_software("unknown version", cfvAMISS);
                        ESP_LOGD(TAG, "UNKNOWN (%.*s)\n", len, data);
                    }
                    timeout_timer_active = false;
                }
                else if (len > 5)
                {                                                            // slave is sending UART information, ignore input and wait until
                    lastVersionRequestTime = esp_timer_get_time() / 1000ULL; // restart timer
                    // change_request_validate_software("noise, version unknown", cfvUNKNOWN);
                }
                // ESP_LOGI(TAG,"%.*s",len, data);
                // printf("%.*s",len, data);
            }

            if ((esp_timer_get_time() / 1000ULL) - lastVersionRequestTime >= REQUEST_FOR_VERSION_INTERVAL)
            {
                lastVersionRequestTime += REQUEST_FOR_VERSION_INTERVAL; // reset timer

                if (version_request_sent == false)
                {
                    version_request_sent = true;
                    ESP_LOGD(TAG, "send UART request for version information");
                    // Write data back to the UART
                    uart_flush_input(VALIDATE_UART_PORT_NUM);
                    uart_write_bytes(VALIDATE_UART_PORT_NUM, "version\n", strlen("version\n"));
                    lastVersionResponseTime = esp_timer_get_time() / 1000ULL; // restart timer
                    timeout_timer_active = true;
                }
                else
                {
                    change_request_validate_software("time-out (4 seconds)", cfvAMISS);
                    timeout_timer_active = false;
                    version_request_sent = false;
                }
            } // timer, request for version

            if (timeout_timer_active == true && (esp_timer_get_time() / 1000ULL) - lastVersionResponseTime >= RESPONSE_FOR_VERSION_TIMEOUT)
            {
                timeout_timer_active = false;
                change_request_validate_software("time-out", cfvAMISS);
            } // timer, response for version
        }
        else
        {

            if (task_restart == false)
            {
                task_restart = true;
                ESP_LOGD(TAG, "task pauze, UART driver turned-off");
                uart_driver_delete(VALIDATE_UART_PORT_NUM);
                change_request_validate_software("task turned off", cfvUNKNOWN);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    } // endless loop
}

void set_icon(enum icon icon_name)
{
#ifdef DISPLAY_ENABLED
    if (!_lcd_ready)
    {
        return;
    }

    _lcd_validate_marquee_enabled = false;

    switch (icon_name)
    {
    case completed:
        lcd_print_centered_line(0, "Flash");
        lcd_print_centered_line(1, "Completed");
        break;
    case failed:
        lcd_print_centered_line(0, "Flash Failed");
        lcd_print_centered_line(1, "Check Target");
        break;
    case progress_1_3:
        lcd_print_centered_line(0, "Flashing...");
        lcd_print_centered_line(1, "Step 1/3");
        break;
    case progress_2_3:
        lcd_print_centered_line(0, "Flashing...");
        lcd_print_centered_line(1, "Step 2/3");
        break;
    case progress_3_3:
        lcd_print_centered_line(0, "Flashing...");
        lcd_print_centered_line(1, "Step 3/3");
        break;
    case open:
        lcd_print_line(0, "Ready to Flash");
        lcd_print_line(1, TARGET_SOFTWARE_VERSION);
        break;
    }
#else
    (void)icon_name;
#endif
}

extern "C" void app_main(void)
{
    static const char *TAG = "main";
    ESP_LOGI(TAG, "Start ...");

    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGD(TAG, "DEBUG LOG ENABLED");

    gpio_config_t io_conf;                      // Fix issue with GPIO 14 (RED LED) didn't work well
    io_conf.intr_type = GPIO_INTR_DISABLE;      // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;            // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // bit mask of the pins that you want to set, e.g.GPIO14/27
    io_conf.pull_down_en = (gpio_pulldown_t)0;  // disable pull-down mode
    io_conf.pull_up_en = (gpio_pullup_t)0;      // disable pull-up mode
    gpio_config(&io_conf);                      // configure GPIO with the given settings

    gpio_set_direction((gpio_num_t)RED_LED_PWR, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)RED_LED_GND, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)GREEN_LED_PWR, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)GREEN_LED_GND, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)RED_LED_PWR, 0);
    gpio_set_level((gpio_num_t)RED_LED_GND, 0);
    gpio_set_level((gpio_num_t)GREEN_LED_PWR, 0);
    gpio_set_level((gpio_num_t)GREEN_LED_GND, 0);

#ifdef DISPLAY_ENABLED
    esp_err_t lcd_err = lcd_init();
    if (lcd_err == ESP_OK)
    {
        _lcd_ready = true;
        lcd_show_boot_text();
        set_icon(open);
    }
    else
    {
        ESP_LOGW(TAG, "LCD init failed (addr 0x%02X, err=%d). Continuing without display.",
                 LCD_I2C_ADDR, (int)lcd_err);
    }
#endif

    target_binaries_t bin; // flash binary configuration

    const loader_esp32_config_t config = {
        .baud_rate = VALIDATE_UART_BAUD_RATE, // 115200
        .uart_port = UART_NUM_1,
        .uart_rx_pin = TARGET_RX_TX,
        .uart_tx_pin = TARGET_TX_RX,
        .reset_trigger_pin = TARGET_RESET,
        .gpio0_trigger_pin = TARGET_IO0,
        .rx_buffer_size = 0,
        .tx_buffer_size = 0,
        .queue_size = 0,
        .uart_queue = NULL,
    };

    // Create binary semaphore for button interrupt
    _button_semaphore = xSemaphoreCreateBinary();

    // Configure button GPIO for interrupt
    gpio_config_t button_io_conf = {};
    button_io_conf.intr_type = GPIO_INTR_NEGEDGE; // Falling edge (button press)
    button_io_conf.mode = GPIO_MODE_INPUT;
    button_io_conf.pin_bit_mask = (1ULL << BUTTON_1);
    button_io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    button_io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&button_io_conf);

    // Install GPIO ISR service
    gpio_install_isr_service(0);

    // Attach ISR handler to button pin
    gpio_isr_handler_add((gpio_num_t)BUTTON_1, button_isr_handler, NULL);

    xTaskCreate(slave_software_validate_task, "software validate task", VALIDATE_TASK_STACK_SIZE, NULL, 10, NULL);

    while (true)
    { // endless loop
        lcd_update_validated_marquee();

        if (_toggle_result_validate_slave_software_changed == true)
        {
            _toggle_result_validate_slave_software_changed = false;

            ESP_LOGI(TAG, "validate slave software, state changed");
            if (_firmware_version == cfvVALIDATED)
            {
                gpio_set_level((gpio_num_t)GREEN_LED_PWR, 1);
                lcd_print_centered_line(0, "Ready to Flash");
                snprintf(_lcd_validate_marquee_text, sizeof(_lcd_validate_marquee_text), "%s    Equal versions", TARGET_SOFTWARE_VERSION);
                _lcd_validate_marquee_offset = 0;
                _lcd_validate_last_step_ms = 0;
                _lcd_validate_marquee_enabled = true;
                lcd_update_validated_marquee();
                ESP_LOGI(TAG, "slave software validated");
            }
            else if (_firmware_version == cfvAMISS)
            {
                gpio_set_level((gpio_num_t)GREEN_LED_PWR, 0);
                set_icon(open);
                ESP_LOGI(TAG, "slave software amiss");
            }
            else
            { // cfvUNKNOWN
                gpio_set_level((gpio_num_t)GREEN_LED_PWR, 0);
                // set_icon(open);
                ESP_LOGI(TAG, "slave software unknown");
            }
        }

        // Wait for button interrupt with 1-second timeout
        if (xSemaphoreTake(_button_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (_button_pressed)
            {
                _button_pressed = false;
                vTaskDelay(pdMS_TO_TICKS(50)); // Debounce delay

                // Confirm button is still pressed after debounce
                if (gpio_get_level((gpio_num_t)BUTTON_1) == 0)
                {

                    ESP_LOGI(TAG, "button pressed");
                    set_icon(progress_1_3);
                    gpio_set_level((gpio_num_t)RED_LED_PWR, 1);
                    gpio_set_level((gpio_num_t)GREEN_LED_PWR, 0);
                    _toggle_validate_slave_software_task_enabled = false;
                    vTaskDelay(100 / portTICK_PERIOD_MS);

                    if (loader_port_esp32_init(&config) != ESP_LOADER_SUCCESS)
                    {
                        ESP_LOGE(TAG, "serial initialization failed");
                        set_icon(failed);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "serial initialization completed");
                        if (connect_to_target(HIGHER_BAUDRATE) != ESP_LOADER_SUCCESS)
                        {
                            ESP_LOGE(TAG, "connect to target failed");
                            set_icon(failed);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "connect to target completed");
                            get_binaries(esp_loader_get_target(), &bin);
                            if (flash_binary(bin.boot.data, bin.boot.size, bin.boot.addr) != ESP_LOADER_SUCCESS)
                            {
                                ESP_LOGE(TAG, "flash binary 'boot' failed");
                                set_icon(failed);
                            }
                            else
                            {
                                ESP_LOGI(TAG, "flash binary 'boot' completed");

                                if (flash_binary(bin.part.data, bin.part.size, bin.part.addr) != ESP_LOADER_SUCCESS)
                                {
                                    ESP_LOGE(TAG, "flash binary 'part' failed");
                                    set_icon(failed);
                                }
                                else
                                {
                                    ESP_LOGI(TAG, "flash binary 'part' completed");
                                    set_icon(progress_2_3);
                                    if (flash_binary(bin.app.data, bin.app.size, bin.app.addr) != ESP_LOADER_SUCCESS)
                                    {
                                        ESP_LOGE(TAG, "flash binary 'app' failed");
                                        set_icon(failed);
                                    }
                                    else
                                    {
                                        ESP_LOGI(TAG, "flash binary 'app' completed");
                                        set_icon(progress_3_3);
#ifdef OTA_ROM
                                        if (flash_binary(bin.ota.data, bin.ota.size, bin.ota.addr) != ESP_LOADER_SUCCESS)
                                        {
                                            ESP_LOGE(TAG, "flash binary 'ota' failed");
                                            set_icon(failed);
                                        }
                                        else
                                        {
                                            ESP_LOGI(TAG, "flash binary 'ota' completed");
                                        } // ota completed
#endif
                                        set_icon(completed);
                                    } // app completed
                                } // part completed
                            } // boot completed
                        } // connect to slave completed
                    } // serial initialization completed

                    ESP_LOGI(TAG, "uart driver delete");
                    uart_driver_delete((uart_port_t)config.uart_port);
                    gpio_set_level((gpio_num_t)RED_LED_PWR, 0);
                    _toggle_validate_slave_software_task_enabled = true;

                } // button confirmed after debounce
            } // button pressed
        } // xSemaphoreTake successful
    } // endless loop
} // app main