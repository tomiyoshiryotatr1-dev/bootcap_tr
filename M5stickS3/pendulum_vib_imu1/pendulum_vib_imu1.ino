#include <M5Unified.h>
#include <Wire.h>
#include <MPU6050.h>
#include <driver/i2s.h>
#include <math.h>

#define I2S_BCK  43
#define I2S_WS   44
#define I2S_DATA 7

#define SAMPLE_RATE 44100
#define FREQ_L 60   // 左(Channel 0)：低め 振動　
#define FREQ_R 60   // 右(Channel 1)：高め

// --- 振り子パラメータ ---
float g = 9.8;
float l1 = 0.1;
float l2 = 0.1;
float m1 = 1.0;
float m2 = 1.0;

// 状態変数
float theta1 = 0.1; //3.14 / 2; // 初期角度を90度に
float theta2 = 0.1; //3.14 / 2;
float omega1 = 0.0;
float omega2 = 0.0;

// float output = 0.0;

// 時間刻み (シミュレーションの精度向上と発散防止のため細かく設定)
float dt = 0.004;

// IMU値 (加速度)
float ax, ay, az;
float max_a = 0.01;
// 描画用スプライト (画面のチラつき防止用バックバッファ)
// M5Canvas sprite(&M5.Display);

MPU6050 mpu;

static float phase_L = 0.0;
static float phase_R = 0.0;

// --- 初期化 ---
void setup() {
  // Serial.begin(9600);
  Serial.begin(115200);
  // auto cfg = M5.config();
  // M5.begin(cfg);

  // M5.Display.setRotation(1); // 画面を横向きに(240x135)
  // M5.Display.fillScreen(BLACK);
  
  // 画面と同じサイズのスプライトを作成
  // sprite.createSprite(240, 135);

  //imu
  
  
  
  Wire.begin(4, 5); // G4, G5

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("ERROR: MPU6050 not connected");
    while (1) {
      delay(1000);
    }
  }
  
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // ★ステレオで送る
    // 通信フォーマットは標準のI2S（Philips規格）にします
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_DATA,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  
  // i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
  // i2s_set_pin(I2S_NUM_1, &pin_config);
  // i2s_zero_dma_buffer(I2S_NUM_1);
  // i2s_config_t i2s_config = {
  //   .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  //   .sample_rate = 44100,
  //   .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  //   .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  //   .communication_format = I2S_COMM_FORMAT_STAND_MSB,
  //   .intr_alloc_flags = 0,
  //   .dma_buf_count = 8,
  //   .dma_buf_len = 64,
  //   .use_apll = false
  // };

  // i2s_pin_config_t pin_config = {
  //   .bck_io_num = I2S_BCK,
  //   .ws_io_num = I2S_WS,
  //   .data_out_num = I2S_DATA,
  //   .data_in_num = I2S_PIN_NO_CHANGE
  // };

  // ★これが重要
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  // i2s_zero_dma_buffer(I2S_NUM_0);
  // while(1){
  //   delay(100);
  //   Serial.println(100);
  // }
}

// --- 二重振り子の更新 ---
void updatePendulum(float input_force) {
  // 二重振り子の運動方程式
  float num1 = -g * (2 * m1 + m2) * sin(theta1);
  float num2 = -m2 * g * sin(theta1 - 2 * theta2);
  float num3 = -2 * sin(theta1 - theta2) * m2;
  float num4 = omega2 * omega2 * l2 + omega1 * omega1 * l1 * cos(theta1 - theta2);
  float den = l1 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));

  float alpha1 = (num1 + num2 + num3 * num4) / den;

  float num5 = 2 * sin(theta1 - theta2);
  float num6 = omega1 * omega1 * l1 * (m1 + m2);
  float num7 = g * (m1 + m2) * cos(theta1);
  float num8 = omega2 * omega2 * l2 * m2 * cos(theta1 - theta2);
  float den2 = l2 * (2 * m1 + m2 - m2 * cos(2 * theta1 - 2 * theta2));

  float alpha2 = (num5 * (num6 + num7 + num8)) / den2;

  // IMUからの外力として加速度を角度の加速度(トルクに相当)として追加
  // これにより端末を振ると振り子が揺さぶられます
  alpha1 += input_force * 1.0; //10
  // alpha2 += input_force * 10.0;

  // 速度と角度の更新（オイラー法）
  omega1 += alpha1 * dt;
  omega2 += alpha2 * dt; 
  

  theta1 += omega1 * dt;
  theta2 += omega2 * dt;

  // 減衰（1に近いほど長引く、小さくするとすぐ止まる）
  // 0.999から0.99に変更し、減衰しやすくしました
  omega1 *= 0.99;
  omega2 *= 0.99;

  // output = alpha2; //alpha2を出力パラメータとする
}

--- 描画 ---
void drawPendulum() {
  // スプライトを黒で塗りつぶす
  sprite.fillSprite(BLACK);

  // 支点の座標 (画面上部中央)
  int cx = 120;
  int cy = 20;

  // 長さをピクセル単位にスケール (0.1m -> 50ピクセル)
  int draw_l1 = 50;
  int draw_l2 = 50;

  int x1 = cx + draw_l1 * sin(theta1);
  int y1 = cy + draw_l1 * cos(theta1);

  int x2 = x1 + draw_l2 * sin(theta2);
  int y2 = y1 + draw_l2 * cos(theta2);

  // 振り子の棒を描画
  sprite.drawLine(cx, cy, x1, y1, WHITE);
  sprite.drawLine(x1, y1, x2, y2, RED);

  // 振り子の質点(重り)と支点を描画
  sprite.fillCircle(cx, cy, 3, GREEN); // 支点
  sprite.fillCircle(x1, y1, 5, WHITE);
  sprite.fillCircle(x2, y2, 5, RED);

  // 入力された加速度の値を画面左上に表示
  sprite.setCursor(5, 5);
  sprite.setTextSize(1);
  sprite.setTextColor(WHITE);
  sprite.printf("In: %.2f", ax);

  // メモリ上のスプライトを画面に一気に転送
  sprite.pushSprite(0, 0);
}

// --- メインループ ---
void loop() {
  // M5.update();

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float norm = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float input = norm - 1.0;

  // IMUから加速度を取得
  // M5.Imu.getAccelData(&ax, &ay, &az);
  // float norm = sqrt(ax*ax + ay*ay + az*az);
  // norm = norm - 1;
  // // 加速度のX軸(端末の長辺方向の傾きや振り)を入力とする
  // float input = norm; //ax 
  if (max_a < input){
    max_a = input;
  }
  if ( input > 5){
    input = 5;
  }
  if (ax_g < 0){
    input = input*(-1);
  }
  // dt=0.004として、0.02秒(20ms)進めるために5回ループして計算
  // こうすることで物理シミュレーションの発散(爆発)を防ぎます
  for(int i = 0; i < 5; i++){
    updatePendulum(input*10);
  }

  // Python側の受信仕様: output,theta1,theta2
  Serial.print(input);
  Serial.print(",");
  Serial.print(theta1);
  Serial.print(",");
  Serial.println(theta2);

  // 描画
  // drawPendulum();

  // 1フレームの待機 (約50fps)
  delay(20);
}