#include <string.h>

#include "driver/i2s.h"
#include "esp_log.h"
#include "sn76489.h"

static const char *TAG = "sn76489_audio";

static constexpr uint32_t AUDIO_SAMPLE_RATE = 22050;
static constexpr size_t AUDIO_SAMPLES = 256;

static bool s_audio_ready = false;

static int16_t s_pcm[AUDIO_SAMPLES];
static uint16_t s_dac[AUDIO_SAMPLES];

esp_err_t sn76489_audio_init(void)
{
    if (s_audio_ready)
        return ESP_OK;

    i2s_config_t cfg = {};

    cfg.mode = (i2s_mode_t)(
        I2S_MODE_MASTER |
        I2S_MODE_TX |
        I2S_MODE_DAC_BUILT_IN
    );
    cfg.sample_rate = AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
    cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
    cfg.intr_alloc_flags = 0;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = 64;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    cfg.fixed_mclk = 0;

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    err = i2s_set_pin(I2S_NUM_0, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_pin: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return err;
    }

    // Canal derecho del DAC interno: GPIO25.
    err = i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_dac_mode: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return err;
    }

    s_audio_ready = true;

    ESP_LOGI(TAG, "Audio I2S/DAC: %lu Hz, GPIO25",
             (unsigned long)AUDIO_SAMPLE_RATE);

    return ESP_OK;
}

void sn76489_audio_push(sn76489_t *psg)
{
    if (!s_audio_ready || !psg)
        return;

    // Trabajar sobre una copia local del estado del PSG.
    sn76489_t local_psg;
    memcpy(&local_psg, psg, sizeof(local_psg));

    if (local_psg.sample_rate != AUDIO_SAMPLE_RATE) {
        local_psg.sample_rate = AUDIO_SAMPLE_RATE;
        local_psg.acc = 0;
    } else if (local_psg.acc >= local_psg.sample_rate) {
        local_psg.acc = 0;
    }

    sn76489_render(&local_psg, s_pcm, AUDIO_SAMPLES);

    // Conservar el estado de los contadores y del generador de ruido.
    memcpy(psg, &local_psg, sizeof(local_psg));

    // El DAC usa los ocho bits más significativos de cada palabra I2S.
    for (size_t i = 0; i < AUDIO_SAMPLES; ++i) {
        uint32_t sample = ((int32_t)s_pcm[i] + 32768) >> 8;
        s_dac[i] = (uint16_t)(sample << 8);
    }

    size_t bytes_written = 0;
    esp_err_t err = i2s_write(
        I2S_NUM_0,
        s_dac,
        sizeof(s_dac),
        &bytes_written,
        portMAX_DELAY
    );

    if (err != ESP_OK || bytes_written != sizeof(s_dac)) {
        ESP_LOGW(TAG, "i2s_write: %s (%u/%u bytes)",
                 esp_err_to_name(err),
                 (unsigned)bytes_written,
                 (unsigned)sizeof(s_dac));
    }
}

void sn76489_audio_deinit(void)
{
    if (!s_audio_ready)
        return;

    i2s_stop(I2S_NUM_0);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_DISABLE);
    i2s_driver_uninstall(I2S_NUM_0);

    s_audio_ready = false;
}

uint32_t sn76489_audio_sample_rate(void)
{
    return AUDIO_SAMPLE_RATE;
}
