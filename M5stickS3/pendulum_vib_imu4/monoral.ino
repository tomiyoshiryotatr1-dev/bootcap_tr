#include <driver/i2s.h>
#include <math.h>
#include <M5Unified.h>

// ============================================================
// BootCap-TR mono / built-in IMU version
//
// 目的:
// - M5StickS3 の内蔵 IMU を使う
// - I2S で振動子へ波形を出す
// - モノラル化のため、L/R に同じ波形を流す
// - EXT_5V 出力を有効化して、外部アンプへ給電できるようにする
//
// 注意:
// - このコードは Hat2-Bus 側の G43 / G44 / G7 を使う
// - 5V は M5.Power.setExtOutput(true) で有効化する
// ============================================================

// ------------------------------
// I2S ピン設定
// ------------------------------
// Hat2-Bus pin 10 = G43
// Hat2-Bus pin 12 = G44
// Hat2-Bus pin 8  = G7
#define I2S_BCK   43
#define I2S_WS    44
#define I2S_DATA  7

// ------------------------------
// オーディオ設定
// ------------------------------
constexpr uint32_t SAMPLE_RATE = 44100;
constexpr float MONO_FREQ_HZ = 60.0f;
constexpr int16_t PCM_PEAK = 5000;
constexpr size_t SAMPLES = 256;

// ------------------------------
// 二重振り子パラメータ
// ------------------------------
float g = 9.8f;
float l1 = 0.01f;
float l2 = 0.01f;
float m1 = 1.5f;
float m2 = 1.5f;

// 状態変数
float theta1 = 0.01f;
float theta2 = 0.01f;
float omega1 = 0.0f;
float omega2 = 0.0f;

// シミュレーション刻み
float dt = 0.004f;

// IMU関連
float ax_f = 0.0f;
float ay_f = 0.0f;
float az_f = 0.0f;

// 出力状態
float output = 0.0f;

// 波形用
static float phaseMono = 0.0f;
int16_t silenceBuffer[SAMPLES * 2];

// ------------------------------------------------------------
// ローパスフィルタ
// ------------------------------------------------------------
float lowpass(float input, float prev, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

// ------------------------------------------------------------
// 二重振り子更新
// input_force を外力として alpha1 に加える
// ------------------------------------------------------------
void updatePendulum(float input_force) {
  float num1 = -g * (2.0f * m1 + m2) * sin(theta1);
  float num2 = -m2 * g * sin(theta1 - 2.0f * theta2);
  float num3 = -2.0f * sin(theta1 - theta2) * m2;
  float num4 = omega2 * omega2 * l2 + omega1 * omega1 * l1 * cos(theta1 - theta2);
  float den  = l1 * (2.0f * m1 + m2 - m2 * cos(2.0f * theta1 - 2.0f * theta2));

  float alpha1 = (num1 + num2 + num3 * num4) / den;

  float num5 = 2.0f * sin(theta1 - theta2);
  float num6 = omega1 * omega1 * l1 * (m1 + m2);
  float num7 = g * (m1 + m2) * cos(theta1);
  float num8 = omega2 * omega2 * l2 * m2 * cos(theta1 - theta2);
  float den2 = l2 * (2.0f * m1 + m2 - m2 * cos(2.0f * theta1 - 2.0f * theta2));

  float alpha2 = (num5 * (num6 + num7 + num8)) / den2;

  // IMU由来の外力を追加
  alpha1 += input_force * 10.0f;

  // オイラー法
  omega1 += alpha1 * dt;
  omega2 += alpha2 * dt;

  theta1 += omega1 * dt;
  theta2 += omega2 * dt;

  // 減衰
  omega1 *= 0.99f;
  omega2 *= 0.99f;

  // 出力には alpha2 を使う
  output = alpha2;
}

// ------------------------------------------------------------
// I2S 初期化
// ------------------------------------------------------------
bool setupI2S() {
  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.sample_rate = SAMPLE_RATE;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = 0;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = 64;
  i2s_config.use_apll = false;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = I2S_BCK;
  pin_config.ws_io_num = I2S_WS;
  pin_config.data_out_num = I2S_DATA;
  pin_config.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] i2s_driver_install failed: %d\n", (int)err);
    return false;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] i2s_set_pin failed: %d\n", (int)err);
    return false;
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

// ------------------------------------------------------------
// モノラル波形を生成して、L/R両方へ同じ値を入れる
// ------------------------------------------------------------
void writeMonoBuffer(float amplitudeNorm) {
  int16_t buffer[SAMPLES * 2];

  amplitudeNorm = fmaxf(0.0f, fminf(amplitudeNorm, 1.0f));
  int16_t amp = (int16_t)(PCM_PEAK * amplitudeNorm);

  float phaseIncrement = (2.0f * PI * MONO_FREQ_HZ) / SAMPLE_RATE;

  for (size_t i = 0; i < SAMPLES; i++) {
    int16_t sample = (int16_t)(sinf(phaseMono) * amp);

    // dual-mono: L/R に同じ波形を入れる
    buffer[2 * i]     = sample;
    buffer[2 * i + 1] = sample;

    phaseMono += phaseIncrement;
    if (phaseMono >= 2.0f * PI) {
      phaseMono -= 2.0f * PI;
    }
  }

  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytesWritten, 0);
}

// ------------------------------------------------------------
// 無音出力
// ------------------------------------------------------------
void writeSilence() {
  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, silenceBuffer, sizeof(silenceBuffer), &bytesWritten, 0);
}

// ------------------------------------------------------------
// setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

  auto cfg = M5.config();
  M5.begin(cfg);

  // EXT_5V を有効化
  M5.Power.setExtOutput(true);

  Serial.println("BootCap-TR mono / built-in IMU version");
  Serial.println("Using M5StickS3 built-in IMU through M5Unified.");
  Serial.println("EXT_5V output: ENABLED");

  if (!setupI2S()) {
    Serial.println("[FATAL] I2S initialization failed.");
    while (true) {
      delay(1000);
    }
  }

  for (size_t i = 0; i < SAMPLES; i++) {
    silenceBuffer[2 * i] = 0;
    silenceBuffer[2 * i + 1] = 0;
  }
}

// ------------------------------------------------------------
// loop
// ------------------------------------------------------------
void loop() {
  M5.update();

  // 内蔵 IMU 取得
  float accX = 0.0f, accY = 0.0f, accZ = 0.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;

  M5.Imu.getImuData(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);

  // ローパス
  float fc = 5.0f;
  float RC = 1.0f / (2.0f * PI * fc);
  float alpha = dt / (RC + dt);

  ax_f = lowpass(accX, ax_f, alpha);
  ay_f = lowpass(accY, ay_f, alpha);
  az_f = lowpass(accZ, az_f, alpha);

  // accX, accY, accZ は G 単位として扱う
  float norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  float input = norm - 1.0f;

  if (input > 5.0f) {
    input = 5.0f;
  }

  // 符号づけ
  if (accX < 0.0f) {
    input = -input;
  }

  // 振り子更新
  updatePendulum(input * 20.0f);

  // シリアル出力
  Serial.print(output);
  Serial.print(",");
  Serial.print(theta1);
  Serial.print(",");
  Serial.println(theta2);

  delay(2);

  // 出力振幅
  float absOutput = fabs(output);
  const float maxOutput = 400.0f;

  if (absOutput > maxOutput) {
    absOutput = maxOutput;
  }

  if (absOutput > 10.0f) {
    float ampNorm = (absOutput * absOutput) / (maxOutput * maxOutput);
    writeMonoBuffer(ampNorm);
  } else {
    writeSilence();
  }
}
