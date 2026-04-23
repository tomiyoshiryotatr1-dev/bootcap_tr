#include <driver/i2s.h>
#include <math.h>
// #include <M5Unified.h>
#include <Wire.h>
#include <MPU6050.h>

#define I2S_BCK  43 //red
#define I2S_WS   44 //orenge
#define I2S_DATA 7

#define SAMPLE_RATE 44100
#define FREQ_L 60   // 左(Channel 0)：低め 振動　
#define FREQ_R 60   // 右(Channel 1)：高め

// ---------------------------------------------------------
// 【ハードウェアの設定（MAX98357AのSD_MODEピン）】
// MAX98357AはSD_MODEピンの電圧でL/Rチャンネルを選択します。
// アンプ1 (Channel 0 / Left):  SD_MODEピンを直にVCC (3.3V/5V) に接続
// アンプ2 (Channel 1 / Right): SD_MODEピンを 210kΩ(3.3V時) または 370kΩ(5V時) の抵抗を介してVCCに接続
// 
// ※SD_MODEを未接続(フローティング)にするとLとRが混ざる(L+R)/2になりますので注意。
// ---------------------------------------------------------
float g = 9.8; //9.8;
float l1 = 0.1;
float l2 = 0.1;
float m1 = 0.5;
float m2 = 0.5; //1

// 状態変数
float theta1 = 0.1; //3.14 / 2; // 初期角度を90度に
float theta2 = 0.1; //3.14 / 2;
float omega1 = 0.0;
float omega2 = 0.0;

float dt = 0.04; //0.004;

// IMU値 (加速度)
float ax, ay, az;
float max_a = 0.01;

float output = 0.0;
const int samples = 256;
int16_t buffer_base[samples * 2];

// M5Canvas sprite(&M5.Display);


static float phase_L = 0.0;
static float phase_R = 0.0;

MPU6050 mpu;

//これは2chで二つのアクチュエータを動作させることを目的としている。
void setup() {
  Serial.begin(115200);

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

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  //pendu
  // auto cfg = M5.config();
  // M5.begin(cfg);

  // M5.Display.setRotation(1); // 画面を横向きに(240x135)
  // M5.Display.fillScreen(BLACK);
  
  // // 画面と同じサイズのスプライトを作成
  // sprite.createSprite(240, 135);


  //imu
  Wire.begin(4, 5); // G4, G5

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("ERROR");
    while (1);
  }

  
  for (int i = 0; i < samples; i++) {
    buffer_base[2 * i]     = 0; //(int16_t)(sin(phase_L) * 1000);   // L
    buffer_base[2 * i + 1] = 0;                                 // R
    
    
  }
  
  
}


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
  alpha1 += input_force * 10.0; //10
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

  output = alpha2; //alpha2を出力パラメータとする
}



void loop() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float norm = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float input = norm - 1.0;

  // if (max_a < input){
  //   max_a = input;
  // }
  if ( input > 5){
    input = 5;
  }
  if (ax_g < 0){
    input = input*(-1);
  }
  for(int i = 0; i < 1; i++){
    updatePendulum(input*30); //10
  }
  Serial.print(output);
  Serial.print(",");
  // Serial.print(max_a);
  // Serial.print(",");
  // Serial.print(input);
  // Serial.print(",");
  // Serial.print(ax_g);
  // Serial.print(",");
  // Serial.print(ay_g);
  // Serial.print(",");
  // Serial.println(az_g);
  
  Serial.print(theta1);
  Serial.print(",");
  Serial.println(theta2);

  


  delay(2); //20

  //vib
  const int samples = 256;
  int16_t buffer[samples * 2]; // ★2倍（L+R）

  // 1サンプリングごとの位相の進み幅
  float phase_increment_L = (2.0f * PI * FREQ_L) / SAMPLE_RATE;
  float phase_increment_R = (2.0f * PI * FREQ_R) / SAMPLE_RATE;

  // ---------------------------------------------------
  // 1. Channel 0 (左) だけを振動させる (1秒間)
  // ---------------------------------------------------
  // Serial.println("Channel 0 (L) ON");
  // for (int iter = 0; iter < (SAMPLE_RATE / samples); iter++) { 

  
  
  

  // delay(500);

  // ---------------------------------------------------
  // 2. Channel 1 (右) だけを振動させる (1秒間)
  // ---------------------------------------------------
  // Serial.println("Channel 1 (R) ON");
  // for (int iter = 0; iter < (SAMPLE_RATE / samples); iter++) { 
  //こっち側を制御 抵抗側
  if (50 < output){
    for (int iter = 0; iter < 2; iter++) { 
      for (int i = 0; i < samples; i++) {
        buffer[2 * i]     = 0;                                 // L
        buffer[2 * i + 1] = (int16_t)(sin(phase_R) * 10000);   // R
        
        phase_R += phase_increment_R;
        if (phase_R >= 2.0f * PI) phase_R -= 2.0f * PI;
      }
      size_t bytes_written;
      i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written,0); //portMAX_DELAY = 0
    }
  }else{
    for (int iter = 0; iter < 2; iter++) { 
      
      size_t bytes_written;
      i2s_write(I2S_NUM_0, buffer_base, sizeof(buffer_base), &bytes_written, 0); // portMAX_DELAY
    }
  }


}
// static float phase_L = 0.0;
// static float phase_R = 0.0;

// //これは2chで二つのアクチュエータを動作させることを目的としている。
// void setup() {
//   Serial.begin(115200);

//   i2s_config_t i2s_config = {
//     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
//     .sample_rate = SAMPLE_RATE,
//     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
//     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // ★ステレオで送る
//     // 通信フォーマットは標準のI2S（Philips規格）にします
//     .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
//     .intr_alloc_flags = 0,
//     .dma_buf_count = 8,
//     .dma_buf_len = 64,
//     .use_apll = false
//   };

//   i2s_pin_config_t pin_config = {
//     .bck_io_num = I2S_BCK,
//     .ws_io_num = I2S_WS,
//     .data_out_num = I2S_DATA,
//     .data_in_num = I2S_PIN_NO_CHANGE
//   };

//   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
//   i2s_set_pin(I2S_NUM_0, &pin_config);
// }

// void loop() {
//   const int samples = 256*4;
//   int16_t buffer[samples * 2]; // ★2倍（L+R）

//   // 1サンプリングごとの位相の進み幅
//   float phase_increment_L = (2.0f * PI * FREQ_L) / SAMPLE_RATE;
//   float phase_increment_R = (2.0f * PI * FREQ_R) / SAMPLE_RATE;

//   // ---------------------------------------------------
//   // 1. Channel 0 (左) だけを振動させる (1秒間)
//   // ---------------------------------------------------
//   Serial.println("Channel 0 (L) ON");
//   for (int iter = 0; iter < (SAMPLE_RATE / samples); iter++) { 
//     for (int i = 0; i < samples; i++) {
//       buffer[2 * i]     = (int16_t)(sin(phase_L) * 5000);   // L
//       buffer[2 * i + 1] = 0;                                 // R
      
//       phase_L += phase_increment_L;
//       if (phase_L >= 2.0f * PI) phase_L -= 2.0f * PI;
//     }
//     size_t bytes_written;
//     i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
//   }

//   // delay(500);

//   // ---------------------------------------------------
//   // 2. Channel 1 (右) だけを振動させる (1秒間)
//   // ---------------------------------------------------
//   Serial.println("Channel 1 (R) ON");
//   for (int iter = 0; iter < (SAMPLE_RATE / samples); iter++) { 
//     for (int i = 0; i < samples; i++) {
//       buffer[2 * i]     = 0;                                 // L
//       buffer[2 * i + 1] = (int16_t)(sin(phase_R) * 5000);   // R
      
//       phase_R += phase_increment_R;
//       if (phase_R >= 2.0f * PI) phase_R -= 2.0f * PI;
//     }
//     size_t bytes_written;
//     i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
//   }

// }