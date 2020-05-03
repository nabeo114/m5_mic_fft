#include <M5StickC.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"

#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * 256)
uint8_t BUFFER[READ_LEN] = {0};

int16_t *adcBuffer = NULL;

#define SAMPLING_FREQUENCY (44100 / 2)

const uint16_t FFTsamples = 256;  // サンプル数は2のべき乗
double vReal[FFTsamples];
double vImag[FFTsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);  // FFTオブジェクトを作る

void i2sInit()
{
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = SAMPLING_FREQUENCY,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
   };

   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA;

   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, SAMPLING_FREQUENCY, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_record_task (void* arg)
{   
  size_t bytesread;
  while(1){
    i2s_read(I2S_NUM_0,(char*) BUFFER, READ_LEN, &bytesread, (100 / portTICK_RATE_MS));
    adcBuffer = (int16_t *)BUFFER;

    sample(FFTsamples);
    DCRemoval(vReal, FFTsamples);  // 直流分を除去
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // 窓関数
    FFT.Compute(FFT_FORWARD); // FFT処理(複素数で計算)
    FFT.ComplexToMagnitude(); // 複素数を実数に変換
    double x = FFT.MajorPeak();
    
    M5.Lcd.fillScreen(BLACK);  // 画面をクリア
    drawChart(FFTsamples / 2);
    M5.Lcd.setCursor(30, 0);
    M5.Lcd.printf("Peak: %.1fHz", x);
    
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

void sample(int nsamples) {
  for (int i = 0; i < nsamples; i++) {
    vReal[i] = (double)adcBuffer[i];
    vImag[i] = 0;
  }
}

void DCRemoval(double *vData, uint16_t samples) {
  double mean = 0;
  for (uint16_t i = 0; i < samples; i++) {
    mean += vData[i];
  }
  mean /= samples;
  for (uint16_t i = 0; i < samples; i++) {
    vData[i] -= mean;
  }
}

int X0 = 0;
int Y0 = 10;
int _height = 80 - Y0;
int _width = 160;
float dmax = 10000.0;

void drawChart(int nsamples) {
  int band_width = floor(_width / nsamples);
//  int band_pad = band_width - 1;
  int band_pad = band_width;

  for (int band = 0; band < nsamples; band++) {
    int hpos = band * band_width + X0;
    float d = vReal[band];
    if (d > dmax) d = dmax;
    int h = (int)((d / dmax) * (_height));
    M5.Lcd.fillRect(hpos, _height - h, band_pad, h, WHITE);

    if ((band % (nsamples / 2)) == 0) {
      M5.Lcd.setCursor(hpos, _height + Y0 - 10);
      M5.Lcd.printf("%.1fkHz", ((band * 1.0 * SAMPLING_FREQUENCY) / FFTsamples / 1000));
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK, WHITE);
  M5.Lcd.println("mic test");

  i2sInit();
  xTaskCreate(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  printf("loop cycling\n");
  vTaskDelay(1000 / portTICK_RATE_MS); // otherwise the main task wastes half of the cpu cycles
}
