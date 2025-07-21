#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_timer.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch_cst816s.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "bsp_qmi8658.h"

#include "lvgl.h"
#include "demos/lv_demos.h"

// GNSS相关定义
// 根据ESP32-S3-Touch-LCD-2开发板引脚图配置：
// 左侧引脚25: GPIO43
// 左侧引脚24: GPIO44  
// 左侧引脚23: GPIO45
// 左侧引脚17: GPIO46
// 右侧引脚3: TXD (UART发送)
// 右侧引脚4: RXD (UART接收)
#define GNSS_UART_NUM UART_NUM_2
#define GNSS_UART_TXD_PIN 43  // 左侧引脚25 - GPIO43 (ESP32 TX -> GNSS RX)
#define GNSS_UART_RXD_PIN 44  // 左侧引脚24 - GPIO44 (ESP32 RX -> GNSS TX)
#define GNSS_UART_BAUD_RATE 115200  // 大多数GNSS模块默认115200波特率
#define GNSS_BUFFER_SIZE 1600

// GNSS数据结构定义
typedef struct {
    double Lon;     //GPS Latitude and longitude
    double Lat;
    char Lon_area;
    char Lat_area;
    uint8_t Time_H;   //Time
    uint8_t Time_M;
    uint8_t Time_S;
    uint8_t Status;   //1:Successful positioning 0：Positioning failed
} GNRMC_t;

typedef struct {
    double Lon;
    double Lat;
} Coordinates_t;

#define EXAMPLE_PIN_NUM_SCLK 39
#define EXAMPLE_PIN_NUM_MOSI 38
#define EXAMPLE_PIN_NUM_MISO 40

#define EXAMPLE_SPI_HOST SPI2_HOST

#define EXAMPLE_I2C_NUM 0 // I2C number
#define EXAMPLE_PIN_NUM_I2C_SDA 48
#define EXAMPLE_PIN_NUM_I2C_SCL 47

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (80 * 1000 * 1000)

#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45

#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

#define EXAMPLE_PIN_NUM_BK_LIGHT 1

#define LCD_BL_LEDC_TIMER LEDC_TIMER_0
#define LCD_BL_LEDC_MODE LEDC_LOW_SPEED_MODE

#define LCD_BL_LEDC_CHANNEL LEDC_CHANNEL_0
#define LCD_BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LCD_BL_LEDC_DUTY (1024)                // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LCD_BL_LEDC_FREQUENCY (10000)          // Frequency in Hertz. Set frequency at 5 kHz

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1

static const char *TAG = "lvgl_example";
static lv_indev_drv_t indev_drv; // Input device driver (Touch)
static lv_disp_drv_t disp_drv;   /*Descriptor of a display driver*/
static SemaphoreHandle_t lvgl_api_mux = NULL;

esp_lcd_panel_handle_t panel_handle;
esp_lcd_touch_handle_t tp;

// GNSS相关变量
static GNRMC_t gps_data;
static Coordinates_t baidu_coords;
static char gnss_buffer[GNSS_BUFFER_SIZE];

void lvgl_qmi8658_ui_init(lv_obj_t *parent);

// GNSS相关函数声明
void gnss_init(void);
GNRMC_t gnss_get_gnrmc(void);
Coordinates_t gnss_get_baidu_coordinates(void);
static double transformLat(double x, double y);
static double transformLon(double x, double y);
static Coordinates_t bd_encrypt(Coordinates_t gg);
static Coordinates_t transform(Coordinates_t gps);
void gnss_monitor_task(void *pvParameters);
double nmea_to_degree(double nmea);

// GNSS坐标转换常量
static const double pi = 3.14159265358979324;
static const double a = 6378245.0;
static const double ee = 0.00669342162296594323;
static const double x_pi = 3.14159265358979324 * 3000.0 / 180.0;

bool lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_api_mux);
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_flush_ready(&disp_drv);
    return false;
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    esp_lcd_touch_read_data(tp);
    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void lv_port_disp_init(void)
{
    static lv_disp_draw_buf_t draw_buf;
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2,
                          EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES); /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = example_lvgl_flush_cb;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf;

    /*Required for Example 3)*/
    disp_drv.full_refresh = 1;
    // disp_drv.direct_mode = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    // disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

void lv_port_indev_init(void)
{
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    // indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);
}

void display_init(void)
{
    ESP_LOGI(TAG, "SPI BUS init");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EXAMPLE_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Install panel IO");

    esp_lcd_panel_io_handle_t io_handle = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ST7789 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
}

void touch_init(void)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_I2C_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller CST816");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
}

void bsp_brightness_init(void)
{
    gpio_set_direction(EXAMPLE_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, 1);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .timer_num = LCD_BL_LEDC_TIMER,
        .duty_resolution = LCD_BL_LEDC_DUTY_RES,
        .freq_hz = LCD_BL_LEDC_FREQUENCY, // Set output frequency at 5 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LCD_BL_LEDC_MODE,
        .channel = LCD_BL_LEDC_CHANNEL,
        .timer_sel = LCD_BL_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = EXAMPLE_PIN_NUM_BK_LIGHT,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void bsp_brightness_set_level(uint8_t level)
{
    if (level > 100)
    {
        ESP_LOGE(TAG, "Brightness value out of range");
        return;
    }

    uint32_t duty = (level * (LCD_BL_LEDC_DUTY - 1)) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LCD_BL_LEDC_MODE, LCD_BL_LEDC_CHANNEL));

    ESP_LOGI(TAG, "LCD brightness set to %d%%", level);
}

// GNSS相关函数实现
static double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}

static double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

static Coordinates_t bd_encrypt(Coordinates_t gg)
{
    Coordinates_t bd;
    double x = gg.Lon, y = gg.Lat;
    double z = sqrt(x * x + y * y) + 0.00002 * sin(y * x_pi);
    double theta = atan2(y, x) + 0.000003 * cos(x * x_pi);
    bd.Lon = z * cos(theta) + 0.0065;
    bd.Lat = z * sin(theta) + 0.006;
    return bd;
}

static Coordinates_t transform(Coordinates_t gps)
{
    Coordinates_t gg;
    double dLat = transformLat(gps.Lon - 105.0, gps.Lat - 35.0);
    double dLon = transformLon(gps.Lon - 105.0, gps.Lat - 35.0);
    double radLat = gps.Lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    gg.Lat = gps.Lat + dLat;
    gg.Lon = gps.Lon + dLon;
    return gg;
}

void gnss_init(void)
{
    ESP_LOGI(TAG, "Initialize GNSS UART on UART%d, TX: GPIO%d, RX: GPIO%d, Baud: %d", 
             GNSS_UART_NUM, GNSS_UART_TXD_PIN, GNSS_UART_RXD_PIN, GNSS_UART_BAUD_RATE);
    
    const uart_config_t uart_config = {
        .baud_rate = GNSS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(GNSS_UART_NUM, GNSS_BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GNSS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GNSS_UART_NUM, GNSS_UART_TXD_PIN, GNSS_UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 测试UART连接 - 发送一个测试命令
    const char *test_cmd = "$PMTK605*31\r\n";  // 请求版本信息
    uart_write_bytes(GNSS_UART_NUM, test_cmd, strlen(test_cmd));
    ESP_LOGI(TAG, "Sent test command: %s", test_cmd);
    
    // 等待一下看看是否有响应
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    char test_buffer[256];
    int test_len = uart_read_bytes(GNSS_UART_NUM, (uint8_t*)test_buffer, sizeof(test_buffer)-1, pdMS_TO_TICKS(500));
    if (test_len > 0) {
        test_buffer[test_len] = '\0';
        ESP_LOGI(TAG, "UART test response (%d bytes): %s", test_len, test_buffer);
    } else {
        ESP_LOGW(TAG, "No UART response received during test");
    }
    
    ESP_LOGI(TAG, "GNSS UART initialized");
    
    // 打印引脚连接说明
    ESP_LOGI(TAG, "=== GNSS Module Connection Guide ===");
    ESP_LOGI(TAG, "ESP32-S3-Touch-LCD-2 Board:");
    ESP_LOGI(TAG, "  - Left Pin 25 (GPIO16) -> GNSS Module RX");
    ESP_LOGI(TAG, "  - Left Pin 24 (GPIO17) -> GNSS Module TX");
    ESP_LOGI(TAG, "  - GND -> GNSS Module GND");
    ESP_LOGI(TAG, "  - 3.3V or 5V -> GNSS Module VCC");
    ESP_LOGI(TAG, "=====================================");
}

GNRMC_t gnss_get_gnrmc(void)
{
    uint16_t add = 0, x = 0, y = 0, z = 0, i = 0, l = 0;
    uint32_t Time = 0;
    double times = 1.0;
    long double latitude = 0, longitude = 0;
    
    gps_data.Status = 0;
    gps_data.Time_H = 0;
    gps_data.Time_M = 0;
    gps_data.Time_S = 0;
    gps_data.Lat = 0;
    gps_data.Lon = 0;
    
    // 清空缓冲区
    memset(gnss_buffer, 0, GNSS_BUFFER_SIZE);
    
    // 读取UART数据
    int len = uart_read_bytes(GNSS_UART_NUM, (uint8_t*)gnss_buffer, GNSS_BUFFER_SIZE - 1, pdMS_TO_TICKS(100));
    if (len > 0) {
        gnss_buffer[len] = '\0';
        ESP_LOGI(TAG, "GNSS raw data (%d bytes): %s", len, gnss_buffer);
        
        // 检查是否包含GNRMC语句
        if (strstr(gnss_buffer, "$GNRMC") == NULL && strstr(gnss_buffer, "$GPRMC") == NULL) {
            ESP_LOGW(TAG, "No GNRMC/GPRMC sentence found in data");
        }
    } else {
        ESP_LOGW(TAG, "No GNSS data received (len=%d)", len);
        return gps_data;
    }
    
    add = 0;
    while (add < GNSS_BUFFER_SIZE - 5) {
        if (gnss_buffer[add] == '$' && gnss_buffer[add+1] == 'G' && 
            (gnss_buffer[add+2] == 'N' || gnss_buffer[add+2] == 'P') &&
            gnss_buffer[add+3] == 'R' && gnss_buffer[add+4] == 'M' && gnss_buffer[add+5] == 'C') {
            
            ESP_LOGI(TAG, "Found GNRMC sentence at position %d", add);
            x = 0;
            for (z = 0; x < 12 && (add+z) < GNSS_BUFFER_SIZE; z++) {
                if (gnss_buffer[add+z] == '\0') {
                    break;
                }
                if (gnss_buffer[add+z] == ',') {
                    x++;
                    ESP_LOGI(TAG, "Field %d: ", x);
                    
                    if (x == 1) { // 时间
                        Time = 0;
                        for (i = 0; gnss_buffer[add+z+i+1] != '.' && gnss_buffer[add+z+i+1] != '\0' && gnss_buffer[add+z+i+1] != ','; i++) {
                            Time = (gnss_buffer[add+z+i+1] - '0') + Time * 10;
                        }
                        
                        gps_data.Time_H = Time / 10000 + 8;
                        gps_data.Time_M = Time / 100 % 100;
                        gps_data.Time_S = Time % 100;
                        if (gps_data.Time_H >= 24) {
                            gps_data.Time_H = gps_data.Time_H - 24;
                        }
                        ESP_LOGI(TAG, "Time: %02d:%02d:%02d", gps_data.Time_H, gps_data.Time_M, gps_data.Time_S);
                        
                    } else if (x == 2) { // 定位状态
                        if (gnss_buffer[add+z+1] == 'A') {
                            gps_data.Status = 1;
                            ESP_LOGI(TAG, "Status: A (Valid)");
                        } else {
                            gps_data.Status = 0;
                            ESP_LOGI(TAG, "Status: %c (Invalid)", gnss_buffer[add+z+1]);
                        }
                        
                    } else if (x == 3) { // 纬度
                        latitude = 0;
                        y = 0;
                        for (i = 0; gnss_buffer[add+z+i+1] != ',' && gnss_buffer[add+z+i+1] != '\0'; i++) {
                            if (gnss_buffer[add+z+i+1] == '.') {
                                y = i;
                                continue;
                            }
                            latitude = (gnss_buffer[add+z+i+1] - '0') + latitude * 10;
                        }
                        times = 1.0;
                        for (int j = 0; j < (i - y - 1); j++) {
                            times = times * 10.0;
                        }
                        gps_data.Lat = (double)latitude / (double)times;
                        ESP_LOGI(TAG, "Latitude: %.7f", gps_data.Lat);
                        
                    } else if (x == 4) { // 纬度半球
                        gps_data.Lat_area = gnss_buffer[add+z+1];
                        ESP_LOGI(TAG, "Lat area: %c", gps_data.Lat_area);
                        
                    } else if (x == 5) { // 经度
                        longitude = 0;
                        y = 0;
                        for (i = 0; gnss_buffer[add+z+i+1] != ',' && gnss_buffer[add+z+i+1] != '\0'; i++) {
                            if (gnss_buffer[add+z+i+1] == '.') {
                                y = i;
                                continue;
                            }
                            longitude = (gnss_buffer[add+z+i+1] - '0') + longitude * 10;
                        }
                        times = 1.0;
                        for (int j = 0; j < (i - y - 1); j++) {
                            times = times * 10.0;
                        }
                        gps_data.Lon = (double)longitude / (double)times;
                        ESP_LOGI(TAG, "Longitude: %.7f", gps_data.Lon);
                        
                    } else if (x == 6) { // 经度半球
                        gps_data.Lon_area = gnss_buffer[add+z+1];
                        ESP_LOGI(TAG, "Lon area: %c", gps_data.Lon_area);
                    }
                }
            }
            ESP_LOGI(TAG, "Final GPS data - Status: %d, Lat: %.7f, Lon: %.7f", 
                     gps_data.Status, gps_data.Lat, gps_data.Lon);
            break;
        }
        if (gnss_buffer[add+5] == '\0') {
            break;
        }
        add++;
    }
    
    return gps_data;
}

Coordinates_t gnss_get_baidu_coordinates(void)
{
    Coordinates_t temp;
    temp.Lat = ((int)(gps_data.Lat)) + (gps_data.Lat - ((int)(gps_data.Lat))) * 100 / 60;
    temp.Lon = ((int)(gps_data.Lon)) + (gps_data.Lon - ((int)(gps_data.Lon))) * 100 / 60;
    temp = transform(temp);
    temp = bd_encrypt(temp);
    return temp;
}

void lvgl_tick_timer_init(uint32_t ms)
{
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, ms * 1000));
}

static void task(void *param)
{
    // ESP_LOGI(TAG, "run");
    while (1)
    {
        uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        while (1)
        {
            // Lock the mutex due to the LVGL APIs are not thread-safe
            if (lvgl_lock(-1))
            {
                task_delay_ms = lv_timer_handler();
                // Release the mutex
                lvgl_unlock();
            }
            if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
            {
                task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
            }
            else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
            {
                task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
            }
            vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
        }
    }
}

void app_main(void)
{
    lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
    lv_init();
    display_init();
    touch_init();
    bsp_qmi8658_init();
    gnss_init();  // 初始化GNSS
    lv_port_disp_init();
    lv_port_indev_init();
    lvgl_tick_timer_init(EXAMPLE_LVGL_TICK_PERIOD_MS);
    bsp_brightness_init();
    bsp_brightness_set_level(80);
    if (lvgl_lock(-1))
    {
        lvgl_qmi8658_ui_init(lv_scr_act());
        // lv_demo_widgets();
        // lv_demo_benchmark();
        // lv_demo_keypad_encoder();
        // lv_demo_music();
        // lv_demo_stress();
        lvgl_unlock();
    }
    xTaskCreatePinnedToCore(task, "bsp_lv_port_task", 1024 * 20, NULL, 5, NULL, 1);
    
    // 创建一个GNSS监控任务
    xTaskCreatePinnedToCore(gnss_monitor_task, "gnss_monitor", 4096, NULL, 4, NULL, 0);
}

lv_obj_t *label_accel_x;
lv_obj_t *label_accel_y;
lv_obj_t *label_accel_z;
lv_obj_t *label_gyro_x;
lv_obj_t *label_gyro_y;
lv_obj_t *label_gyro_z;

// GNSS相关标签
lv_obj_t *label_gps_status;
lv_obj_t *label_gps_time;
lv_obj_t *label_gps_lat;
lv_obj_t *label_gps_lon;

lv_timer_t *qmi8658_timer = NULL;
lv_timer_t *gnss_timer = NULL;

static void qmi8658_callback(lv_timer_t *timer)
{
    qmi8658_data_t data;
    bsp_qmi8658_read_data(&data);
    lv_label_set_text_fmt(label_accel_x, "%d", data.acc_x);
    lv_label_set_text_fmt(label_accel_y, "%d", data.acc_y);
    lv_label_set_text_fmt(label_accel_z, "%d", data.acc_z);
    lv_label_set_text_fmt(label_gyro_x, "%d", data.gyr_x);
    lv_label_set_text_fmt(label_gyro_y, "%d", data.gyr_y);
    lv_label_set_text_fmt(label_gyro_z, "%d", data.gyr_z);
}

static void gnss_callback(lv_timer_t *timer)
{
    GNRMC_t gps = gnss_get_gnrmc();
    
    ESP_LOGI(TAG, "GNSS callback - Status: %d, Lat: %.7f, Lon: %.7f, Time: %02d:%02d:%02d", 
             gps.Status, gps.Lat, gps.Lon, gps.Time_H, gps.Time_M, gps.Time_S);
    
    // 更新GPS状态
    if (gps.Status) {
        lv_label_set_text(label_gps_status, "locate success");
        lv_obj_set_style_text_color(label_gps_status, lv_color_make(0, 255, 0), 0);
        
        // 定位成功时更新坐标
        // 删除百度坐标相关内容，只保留GPS经纬度显示
        char buf[32];
        snprintf(buf, sizeof(buf), "%d.%04d", (int)nmea_to_degree(gps.Lat), (int)((nmea_to_degree(gps.Lat) - (int)nmea_to_degree(gps.Lat)) * 1000000));
        lv_label_set_text(label_gps_lat, buf);
        snprintf(buf, sizeof(buf), "%d.%04d", (int)nmea_to_degree(gps.Lon), (int)((nmea_to_degree(gps.Lon) - (int)nmea_to_degree(gps.Lon)) * 1000000));
        lv_label_set_text(label_gps_lon, buf);
        
        ESP_LOGI(TAG, "GPS located successfully - WGS84 Lat: %.7f, Lon: %.7f", nmea_to_degree(gps.Lat), nmea_to_degree(gps.Lon));
    } else {
        // 定位失败时显示搜索状态，坐标保持上次有效值
        static uint8_t dot_count = 0;
        char searching_text[20];
        snprintf(searching_text, sizeof(searching_text), "searching");
        for (int i = 0; i < dot_count; i++) {
            strcat(searching_text, ".");
        }
        lv_label_set_text(label_gps_status, searching_text);
        lv_obj_set_style_text_color(label_gps_status, lv_color_make(255, 165, 0), 0); // 橙色
        
        // 更新点号动画
        dot_count = (dot_count + 1) % 4;
        
        ESP_LOGW(TAG, "GPS still searching...");
    }
    
    // 无论定位是否成功都更新时间（如果时间有效）
    if (gps.Time_H != 0 || gps.Time_M != 0 || gps.Time_S != 0) {
        lv_label_set_text_fmt(label_gps_time, "%02d:%02d:%02d", gps.Time_H, gps.Time_M, gps.Time_S);
    }
}

void lvgl_qmi8658_ui_init(lv_obj_t *parent)
{
    lv_obj_t *list = lv_list_create(parent);
    lv_obj_set_size(list, lv_pct(100), lv_pct(100));

    // IMU数据
    lv_obj_t *list_item = lv_list_add_btn(list, NULL, "accel_x");
    label_accel_x = lv_label_create(list_item);
    lv_label_set_text(label_accel_x, "0");

    list_item = lv_list_add_btn(list, NULL, "accel_y");
    label_accel_y = lv_label_create(list_item);
    lv_label_set_text(label_accel_y, "0");

    list_item = lv_list_add_btn(list, NULL, "accel_z");
    label_accel_z = lv_label_create(list_item);
    lv_label_set_text(label_accel_z, "0");

    list_item = lv_list_add_btn(list, NULL, "gyro_x");
    label_gyro_x = lv_label_create(list_item);
    lv_label_set_text(label_gyro_x, "0");

    list_item = lv_list_add_btn(list, NULL, "gyro_y");
    label_gyro_y = lv_label_create(list_item);
    lv_label_set_text(label_gyro_y, "0");

    list_item = lv_list_add_btn(list, NULL, "gyro_z");
    label_gyro_z = lv_label_create(list_item);
    lv_label_set_text(label_gyro_z, "0");

    // GNSS数据
    list_item = lv_list_add_btn(list, NULL, "GPS status");
    label_gps_status = lv_label_create(list_item);
    lv_label_set_text(label_gps_status, "searching");

    list_item = lv_list_add_btn(list, NULL, "GPS time");
    label_gps_time = lv_label_create(list_item);
    lv_label_set_text(label_gps_time, "00:00:00");

    list_item = lv_list_add_btn(list, NULL, "GPS latitude");
    label_gps_lat = lv_label_create(list_item);
    lv_label_set_text(label_gps_lat, "0.0000000");

    list_item = lv_list_add_btn(list, NULL, "GPS longitude");
    label_gps_lon = lv_label_create(list_item);
    lv_label_set_text(label_gps_lon, "0.0000000");

    qmi8658_timer = lv_timer_create(qmi8658_callback, 100, NULL);
    gnss_timer = lv_timer_create(gnss_callback, 500, NULL);  // GNSS update frequency 500ms for faster response
}

void gnss_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "GNSS monitor task started");
    
    while (1) {
        // 每5秒检查一次GNSS数据
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // 读取原始UART数据
        char raw_buffer[512];
        int len = uart_read_bytes(GNSS_UART_NUM, (uint8_t*)raw_buffer, sizeof(raw_buffer)-1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            raw_buffer[len] = '\0';
            ESP_LOGI(TAG, "GNSS Monitor - Raw data (%d bytes): %s", len, raw_buffer);
            
            // 检查是否包含各种NMEA语句
            if (strstr(raw_buffer, "$GNRMC") || strstr(raw_buffer, "$GPRMC")) {
                ESP_LOGI(TAG, "Found RMC sentence");
            }
            if (strstr(raw_buffer, "$GNGGA") || strstr(raw_buffer, "$GPGGA")) {
                ESP_LOGI(TAG, "Found GGA sentence");
            }
            if (strstr(raw_buffer, "$GPGSV") || strstr(raw_buffer, "$GNGSV")) {
                ESP_LOGI(TAG, "Found GSV sentence");
            }
        } else {
            ESP_LOGW(TAG, "GNSS Monitor - No data received");
        }
        
        // 测试解析函数
        GNRMC_t test_gps = gnss_get_gnrmc();
        ESP_LOGI(TAG, "GNSS Monitor - Parsed data: Status=%d, Lat=%.7f, Lon=%.7f, Time=%02d:%02d:%02d", 
                 test_gps.Status, test_gps.Lat, test_gps.Lon, test_gps.Time_H, test_gps.Time_M, test_gps.Time_S);
    }
}

double nmea_to_degree(double nmea)
{
    int degree = (int)(nmea / 100);
    double minute = nmea - degree * 100;
    return degree + minute / 60.0;
}
