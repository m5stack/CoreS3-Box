/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_check.h"
#include "hal/i2s_hal.h"

#include "bsp_board.h"
#include "iot_button.h"
#include "es7210.h"

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

#define ES8311_SAMPLE_RATE          (16000)
#define ES8311_DEFAULT_VOLUME       (60)

#define ES7210_I2C_ADDR             (0x40)
#define ES7210_SAMPLE_RATE          (16000)
#define ES7210_I2S_FORMAT           ES7210_I2S_FMT_I2S
#define ES7210_MCLK_MULTIPLE        (256)
#define ES7210_BIT_WIDTH            ES7210_I2S_BITS_16B
#define ES7210_MIC_BIAS             ES7210_MIC_BIAS_2V87
#define ES7210_MIC_GAIN             ES7210_MIC_GAIN_30DB
#define ES7210_ADC_VOLUME           (0)

static const pmod_pins_t g_pmod[2] = {
    {
        {BSP_PMOD2_IO5, BSP_PMOD2_IO6, BSP_PMOD2_IO7, BSP_PMOD2_IO8},
        {BSP_PMOD2_IO1, BSP_PMOD2_IO2, BSP_PMOD2_IO3, BSP_PMOD2_IO4},
    },
    {
        {BSP_PMOD1_IO5, BSP_PMOD1_IO6, BSP_PMOD1_IO7, BSP_PMOD1_IO8},
        {BSP_PMOD1_IO1, BSP_PMOD1_IO2, BSP_PMOD1_IO3, BSP_PMOD1_IO4},
    },
};

static const board_res_desc_t g_board_m5stack_cores3_res = {

    .FUNC_SDMMC_EN =   (0),
    .SDMMC_BUS_WIDTH = (4),
    .GPIO_SDMMC_CLK =  (GPIO_NUM_NC),
    .GPIO_SDMMC_CMD =  (GPIO_NUM_NC),
    .GPIO_SDMMC_D0 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_D1 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_D2 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_D3 =   (GPIO_NUM_NC),
    .GPIO_SDMMC_DET =  (GPIO_NUM_NC),

    .FUNC_SDSPI_EN =       (0),
    .SDSPI_HOST =          (SPI2_HOST),
    .GPIO_SDSPI_CS =       (GPIO_NUM_NC),
    .GPIO_SDSPI_SCLK =     (GPIO_NUM_NC),
    .GPIO_SDSPI_MISO =     (GPIO_NUM_NC),
    .GPIO_SDSPI_MOSI =     (GPIO_NUM_NC),

    .FUNC_SPI_EN =         (0),
    .GPIO_SPI_CS =         (GPIO_NUM_NC),
    .GPIO_SPI_MISO =       (GPIO_NUM_NC),
    .GPIO_SPI_MOSI =       (GPIO_NUM_NC),
    .GPIO_SPI_SCLK =       (GPIO_NUM_NC),

    .FUNC_RMT_EN =         (0),
    .GPIO_RMT_IR =         (GPIO_NUM_NC),
    .GPIO_RMT_LED =        (GPIO_NUM_NC),

    .PMOD1 = &g_pmod[0],
    .PMOD2 = &g_pmod[1],
};

static bsp_codec_config_t g_codec_handle;

static const boards_info_t g_boards_info = {
    .id =           BOARD_M5STACK_CORES3,
    .name =         "M5Stack CoreS3",
    .board_desc =   &g_board_m5stack_cores3_res
};

static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;

static es7210_dev_handle_t es7210_handle = NULL;

static const char *TAG = "board";

static esp_err_t bsp_i2s_read(void *audio_buffer, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2s_channel_read(i2s_rx_chan, (char *)audio_buffer, len, bytes_read, timeout_ms);
    return ret;
}

static esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2s_channel_write(i2s_tx_chan, (char *)audio_buffer, len, bytes_written, timeout_ms);
    return ret;
}

static esp_err_t bsp_i2s_reconfig_clk(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)bits_cfg, (i2s_slot_mode_t)ch),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };

    ret |= i2s_channel_disable(i2s_tx_chan);
    ret |= i2s_channel_reconfig_std_clock(i2s_tx_chan, &std_cfg.clk_cfg);
    ret |= i2s_channel_reconfig_std_slot(i2s_tx_chan, &std_cfg.slot_cfg);
    ret |= i2s_channel_enable(i2s_tx_chan);
    return ret;
}

static esp_err_t bus_i2c_read(uint8_t addr, const uint8_t out_data, size_t out_size,
                          uint8_t *in_data, size_t in_size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (out_data && out_size) {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, addr << 1, true);
        i2c_master_write(cmd, &out_data, out_size, true);
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | 1, true);
    i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(CONFIG_BSP_I2C_NUM, cmd,
                                         pdMS_TO_TICKS(1000));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d", addr,
                 CONFIG_BSP_I2C_NUM, res);

    i2c_cmd_link_delete(cmd);
    return res;
}

static int bus_bus_i2c_write_16(uint8_t slv_addr, uint8_t reg, uint16_t data)
{
    esp_err_t ret = ESP_FAIL;
    uint16_t data_htons = LITTLETOBIG(data);
    uint8_t *data_u8 = (uint8_t *)&data_htons;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_MASTER_WRITE, 0x01);
    i2c_master_write_byte(cmd, reg, 0x01);
    i2c_master_write_byte(cmd, data_u8[0], 0x01);
    i2c_master_write_byte(cmd, data_u8[1], 0x01);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(CONFIG_BSP_I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

static uint16_t bus_i2c_read_16(uint8_t slv_addr, uint8_t reg)
{
    uint16_t data = 0;
    uint8_t *data_u8 = (uint8_t *)&data;
    esp_err_t ret = ESP_FAIL;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_MASTER_WRITE, 0x01);
    i2c_master_write_byte(cmd, reg, 0x01);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(CONFIG_BSP_I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) return -1;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( slv_addr << 1 ) | I2C_MASTER_READ, 0x01);
    i2c_master_read_byte(cmd, &data_u8[1], 0x00);
    i2c_master_read_byte(cmd, &data_u8[0], 0x01);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(CONFIG_BSP_I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return data;
}

static esp_err_t bsp_codec_volume_set(int volume, int *volume_set)
{   
    esp_err_t ret = ESP_OK;
    int v = 10 - volume / 10;
    if ( v == 10)
    {
        bus_bus_i2c_write_16(0x36, 0x0C, 0xFF64);
        bus_bus_i2c_write_16(0x36, 0x05, 0x0018);
        ESP_LOGI(TAG, "%d", v);
        ESP_LOGI(TAG, "%X", 0xFF64);
    }
    else
    {
        bus_bus_i2c_write_16(0x36, 0x05, 0x0008);
        bus_bus_i2c_write_16(0x36, 0x0C, v * 11 * 256 + 100);
        ESP_LOGI(TAG, "%d", v);
        ESP_LOGI(TAG, "%X", v * 11 * 256 + 100);
    }
    return ESP_OK;
}

static esp_err_t bsp_codec_mute_set(bool enable)
{
    bus_bus_i2c_write_16(0x36, 0x0C, 0xFF64);
    bus_bus_i2c_write_16(0x36, 0x05, 0x0018);
    return ESP_OK;
}

static esp_err_t bsp_codec_es7210_set()
{
    esp_err_t ret = ESP_OK;

    es7210_codec_config_t codec_conf = {
        .i2s_format = ES7210_I2S_FORMAT,
        .mclk_ratio = ES7210_MCLK_MULTIPLE,
        .sample_rate_hz = ES7210_SAMPLE_RATE,
        .bit_width = ES7210_BIT_WIDTH,
        .mic_bias = ES7210_MIC_BIAS,
        .mic_gain = ES7210_MIC_GAIN,
        .flags.tdm_enable = false
    };
    ret |= es7210_config_codec(es7210_handle, &codec_conf);
    ret |= es7210_config_volume(es7210_handle, ES7210_ADC_VOLUME);
    return ret;
}

static void bsp_codec_init()
{
    /* Create ES7210 device handle */
    es7210_i2c_config_t es7210_i2c_conf = {
        .i2c_port = BSP_I2C_NUM,
        .i2c_addr = ES7210_I2C_ADDR
    };
    es7210_new_codec(&es7210_i2c_conf, &es7210_handle);
    bsp_codec_es7210_set();

    /* Configure I2S peripheral and Power Amplifier */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = BSP_I2S_GPIO_CFG,
    };
    bsp_audio_init(&std_cfg, &i2s_tx_chan, &i2s_rx_chan);
    bsp_audio_poweramp_enable(true);

    bsp_codec_config_t *codec_config = bsp_board_get_codec_handle();
    codec_config->volume_set_fn = bsp_codec_volume_set;
    codec_config->mute_set_fn = bsp_codec_mute_set;
    codec_config->codec_reconfig_fn = bsp_codec_es7210_set;
    codec_config->i2s_read_fn = bsp_i2s_read;
    codec_config->i2s_write_fn = bsp_i2s_write;
    codec_config->i2s_reconfig_clk_fn = bsp_i2s_reconfig_clk;
}

__attribute__((weak)) void mute_btn_handler(void *handle, void *arg)
{
    button_event_t event = (button_event_t)arg;

    if (BUTTON_PRESS_DOWN == event) {
        esp_rom_printf(DRAM_STR("Mute On\r\n"));
    } else {
        esp_rom_printf(DRAM_STR("Mute Off\r\n"));
    }
}

static esp_err_t bus_i2c_write(uint8_t addr, const uint8_t out_reg,
                           size_t out_reg_size, const uint8_t out_data,
                           size_t out_size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1, true);
    if (out_reg && out_reg_size)
        i2c_master_write(cmd, &out_reg, out_reg_size, true);
    i2c_master_write(cmd, &out_data, out_size, true);
    i2c_master_stop(cmd);
    esp_err_t res =
        i2c_master_cmd_begin(CONFIG_BSP_I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    if (res != ESP_OK)
        ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d", addr, 0,
                 res);
    i2c_cmd_link_delete(cmd);

    return res;
}

static void aw9523_init(void) {
    // aw9523 reset
    bus_i2c_write(0x58, 0x7F, 1, 0x00, 1);
    vTaskDelay(30);
    // aw9523 default seetting
    bus_i2c_write(0x58, 0x04, 1, 0b11011000, 1);
    bus_i2c_write(0x58, 0x05, 1, 0b01111100, 1);
    bus_i2c_write(0x58, 0x12, 1, 0b11111111, 1);
    bus_i2c_write(0x58, 0x13, 1, 0b11111111, 1);
    bus_i2c_write(0x58, 0x11, 1, (1 << 4), 1);
    bus_i2c_write(0x58, 0x02, 1, 0b00000101, 1);
    bus_i2c_write(0x58, 0x03, 1, 0b00000011, 1);
}

static void aw88298_init(void) {
    bus_bus_i2c_write_16(0x36, 0x61, 0x0673);
    bus_bus_i2c_write_16(0x36, 0x04, 0x4040);
    bus_bus_i2c_write_16(0x36, 0x05, 0x0008);
    bus_bus_i2c_write_16(0x36, 0x06, 0x14C8);
    bus_bus_i2c_write_16(0x36, 0x0C, 0x0064);
}

static void axp2101_turn_on_bl(void) {
    uint8_t cfg = 0;
    bus_i2c_read(0x34, 0x90, 1, &cfg, 1);
    bus_i2c_write(0x34, 0x90, 1, cfg | 0x80, 1);
    // from 0.5V to 3.5V 100mv/step
    // 0b00000  0.5V
    // 0b11110  3.5V
    bus_i2c_write(0x34, 0x99, 1, 0b11110 - 5, 1);  // DLDO1
}

esp_err_t bsp_m5stack_cores3_init(void)
{
    aw9523_init();
    vTaskDelay(100);
    axp2101_turn_on_bl();
    aw88298_init();

    /**
     * @brief Initialize I2S and audio codec
     *
     * @note Actually the sampling rate can be reconfigured.
     *       `MP3GetLastFrameInfo` can fill the `MP3FrameInfo`, which includes `samprate`.
     *       So theoretically, the sampling rate can be dynamically changed according to the MP3 frame information.
     */
    bsp_codec_init();

    return ESP_OK;
}

const boards_info_t *bsp_board_get_info(void)
{
    return &g_boards_info;
}

const board_res_desc_t *bsp_board_get_description(void)
{
    return g_boards_info.board_desc;
}

bsp_codec_config_t *bsp_board_get_codec_handle(void)
{
    return &g_codec_handle;
}

esp_err_t bsp_board_init(void)
{
    esp_err_t ret = ESP_OK;

    ret |= bsp_m5stack_cores3_init();

    return ret;
}
