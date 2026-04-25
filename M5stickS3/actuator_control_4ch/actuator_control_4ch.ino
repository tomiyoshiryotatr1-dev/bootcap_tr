#include <driver/i2s.h>
#include <math.h>

// ============================================================================
// M5StickS3 用 4ch アクチュエータ制御スケッチ（教育用コメント多め版）
//
// 【このスケッチの目的】
//   4個の I2S 対応アンプ / アクチュエータを、M5StickS3 から安定して駆動する。
//   ただし、最初から複雑な表現を狙うのではなく、まずは
//   「4ch が独立して正しく鳴ることを確認する」ことを最優先にする。
//
// 【構成】
//   - I2S_NUM_0 のステレオ出力を 2ch 分として使う
//       L = ch0, R = ch1
//   - I2S_NUM_1 のステレオ出力を 2ch 分として使う
//       L = ch2, R = ch3
//
//   つまり、1本の stereo I2S × 2系統 = 合計4ch という構成である。
//
// 【重要な考え方】
//   このスケッチは「完成版の触覚レンダラ」ではなく、
//   あくまで bring-up（立ち上げ確認）とデバッグのための土台である。
//   そのため、以下を重視している。
//   1. 配線ミスが見つけやすいこと
//   2. どこで失敗したか分かりやすいこと
//   3. 後から IMU や物理モデルを繋ぎやすいこと
//
// 【推奨ピン】
//   I2S0: BCK=G5,  WS=G6,  DATA=G7
//   I2S1: BCK=G8,  WS=G9,  DATA=G10
//
//   M5StickS3 では G1, G2, G4 などがオンボード機能と結びついているため、
//   I2S の外部出力用ピンとしては避けた方が安全である。
//
// 【電源について】
//   外部アンプを M5StickS3 の EXT_5V / Grove 5V から直接給電する場合、
//   ボード側の 5V 出力設定や電流容量に注意が必要である。
//   4ch をしっかり駆動したい場合は、外部 5V 電源を別途用意する方が安全。
// ============================================================================

namespace audio4ch {

// ============================================================================
// 1. 基本設定
// ============================================================================

// サンプルレート
// 低周波の触覚駆動確認が目的なので、44.1kHz ではなく 16kHz に落としている。
// これにより CPU / DMA 負荷が少し下がり、後で IMU や制御ロジックを足しやすい。
constexpr uint32_t SAMPLE_RATE = 16000;

// 1回の i2s_write で流すフレーム数。
// 大きすぎると応答性が下がり、小さすぎるとオーバーヘッドが増える。
// まずは 128 サンプル程度から始めるのが扱いやすい。
constexpr size_t FRAMES_PER_BUFFER = 128;

// i2s_write のタイムアウト。
// 無限待ちにすると、片系統が止まったときに全体の切り分けがしにくい。
// そのため、短めのタイムアウトを入れている。
constexpr TickType_t I2S_TIMEOUT_TICKS = pdMS_TO_TICKS(20);

// 2π はサイン波生成で毎回使うため、定数として定義しておく。
constexpr float TWO_PI_F = 6.2831853071795864769f;

// 16bit PCM の最大振幅に対して、少し余裕を持たせたピーク値。
// 32767 いっぱいまで使うと歪みやすく、デバッグ中は音量も大きすぎる。
// まずは控えめな値にしておく。
constexpr int16_t PCM_PEAK = 12000;


// ============================================================================
// 2. テストモード設定
// ============================================================================

// このスケッチでは、まず 4ch 同時出力ではなく、
// 1ch ずつ順番に鳴らす自己診断モードを用意している。
// これにより、配線ミスや L/R の取り違えを見つけやすい。
enum class TestMode : uint8_t {
  SequentialSingleChannel,   // ch0 → ch1 → ch2 → ch3 の順で1本ずつ鳴らす
  AllChannelsDifferentFreq   // 4ch を同時に、別々の周波数で鳴らす
};

// 初期状態では自己診断を優先する。
constexpr TestMode TEST_MODE = TestMode::SequentialSingleChannel;

// 何ミリ秒ごとに、次のチャンネルへ切り替えるか。
constexpr uint32_t STEP_MS = 1200;


// ============================================================================
// 3. データ構造
// ============================================================================

// 1チャンネル分の状態。
// 後で「周波数だけでなく振幅も動的に変える」「物理モデルで変調する」などの拡張を
// しやすいように、状態をまとめて構造体にしている。
struct ChannelState {
  float freq_hz;      // このチャンネルの出力周波数 [Hz]
  float amplitude;    // 振幅（0.0 ～ 1.0）
  float phase;        // 現在の位相 [rad]
  bool enabled;       // このチャンネルを鳴らすかどうか
};

// I2S のピン3本をまとめた構造体。
// 「どのポートにどのピンを割り当てているか」を読みやすくするために分離している。
struct I2SPins {
  int bck;    // ビットクロック
  int ws;     // LRCK / Word Select
  int data;   // データ出力
};

// 1本の I2S バス（= stereo 2ch）に対応する構造体。
// left と right が、それぞれ左チャネル / 右チャネルに対応する。
// buffer はこのバスにまとめて流す PCM バッファである。
struct BusState {
  i2s_port_t port;            // I2S_NUM_0 または I2S_NUM_1
  I2SPins pins;               // このバスに使う物理ピン
  ChannelState* left;         // 左チャネルに対応する状態
  ChannelState* right;        // 右チャネルに対応する状態
  int16_t buffer[FRAMES_PER_BUFFER * 2];  // L/R interleaved な PCM バッファ
};


// ============================================================================
// 4. チャンネル設定
// ============================================================================

// 4ch ぶんの初期設定。
// 周波数を全部同じにすると、どのチャンネルが鳴っているか分かりにくい。
// そのため、同時出力モードでも区別しやすいよう、あらかじめ少しずつ変えてある。
ChannelState channels[4] = {
  {  60.0f, 0.35f, 0.0f, true  },   // ch0
  {  90.0f, 0.35f, 0.0f, false },   // ch1
  { 120.0f, 0.35f, 0.0f, false },   // ch2
  { 150.0f, 0.35f, 0.0f, false }    // ch3
};

// I2S0 は ch0 / ch1 を担当する。
// G5, G6, G7 は Hat 側で比較的扱いやすい。
BusState bus0 = {
  I2S_NUM_0,
  {5, 6, 7},
  &channels[0],
  &channels[1],
  {0}
};

// I2S1 は ch2 / ch3 を担当する。
// G8 は Hat 側、G9/G10 は HY2.0-4P 側に出ている。
// 実配線は少し面倒だが、G1/G2/G4 よりオンボード機能との衝突が少ない。
BusState bus1 = {
  I2S_NUM_1,
  {8, 9, 10},
  &channels[2],
  &channels[3],
  {0}
};

// テストモード用の状態変数。
// millis() を使って一定時間ごとに「今どのチャンネルを鳴らすか」を切り替える。
uint32_t last_step_ms = 0;
uint8_t active_channel = 0;


// ============================================================================
// 5. 小さなユーティリティ関数
// ============================================================================

// 0.0 ～ 1.0 の範囲に値を収める関数。
// 振幅が誤って負や1超えになったときに、破綻しにくくするための保険。
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// -1.0 ～ +1.0 の正規化浮動小数点値を 16bit PCM に変換する。
// 実際に I2S へ送るのは整数の PCM 値なので、ここで変換する。
int16_t toPcm16(float normalized) {
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  return static_cast<int16_t>(normalized * PCM_PEAK);
}

// シリアルへ I2S エラーを表示する補助関数。
// どのポートで何が失敗したのかを、最低限すぐ分かるようにしている。
void logI2SError(const char* label, i2s_port_t port, esp_err_t err) {
  Serial.printf("[ERROR] %s (port=%d, err=%d)\n",
                label,
                static_cast<int>(port),
                static_cast<int>(err));
}


// ============================================================================
// 6. 波形生成
// ============================================================================

// 1チャンネルぶんの「次のサンプル値」を生成する。
// 今回は最も単純なサイン波にしている。
// 後で物理モデルやノイズ、包絡線などを入れたくなったら、この関数を育てていけばよい。
float nextSample(ChannelState& ch) {
  // 無効化されているチャンネルは無音にする。
  if (!ch.enabled || ch.amplitude <= 0.0f || ch.freq_hz <= 0.0f) {
    return 0.0f;
  }

  // 現在位相からサイン波を計算し、振幅を掛ける。
  const float out = sinf(ch.phase) * clamp01(ch.amplitude);

  // 位相を1サンプル分だけ進める。
  // 位相増分 = 2πf / Fs
  ch.phase += TWO_PI_F * ch.freq_hz / static_cast<float>(SAMPLE_RATE);

  // 位相が 2π を超えたら1周ぶん戻す。
  if (ch.phase >= TWO_PI_F) {
    ch.phase -= TWO_PI_F;
  }

  return out;
}


// ============================================================================
// 7. I2S 初期化
// ============================================================================

// 1本の I2S バスを初期化する関数。
// 成功したら true、失敗したら false を返す。
bool setupI2S(BusState& bus) {
  // まず I2S の基本設定を作る。
  // 今回は legacy driver を使って、元コードとの差分を小さくしている。
  i2s_config_t cfg = {};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = FRAMES_PER_BUFFER;
  cfg.use_apll = false;

  // ピン設定。
  // data_in は使わないので I2S_PIN_NO_CHANGE にしている。
  i2s_pin_config_t pin_cfg = {};
  pin_cfg.bck_io_num = bus.pins.bck;
  pin_cfg.ws_io_num = bus.pins.ws;
  pin_cfg.data_out_num = bus.pins.data;
  pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;

  // ドライバをインストールする。
  esp_err_t err = i2s_driver_install(bus.port, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    logI2SError("i2s_driver_install failed", bus.port, err);
    return false;
  }

  // 物理ピンを割り当てる。
  err = i2s_set_pin(bus.port, &pin_cfg);
  if (err != ESP_OK) {
    logI2SError("i2s_set_pin failed", bus.port, err);
    return false;
  }

  // DMA バッファを 0 クリアしておく。
  // これをしておくと、立ち上がり時に変なゴミが出る確率を少し下げられる。
  err = i2s_zero_dma_buffer(bus.port);
  if (err != ESP_OK) {
    logI2SError("i2s_zero_dma_buffer failed", bus.port, err);
    return false;
  }

  // 初期化成功の表示。
  Serial.printf("[OK] I2S port %d started (BCK=%d WS=%d DATA=%d)\n",
                static_cast<int>(bus.port),
                bus.pins.bck,
                bus.pins.ws,
                bus.pins.data);

  return true;
}


// ============================================================================
// 8. PCM バッファ生成
// ============================================================================

// 1本の I2S バスについて、L/R 2ch ぶんの PCM バッファを埋める。
// buffer は [L0, R0, L1, R1, L2, R2, ...] の interleaved 形式になる。
void fillBusBuffer(BusState& bus) {
  for (size_t i = 0; i < FRAMES_PER_BUFFER; ++i) {
    // 左右それぞれの「次のサンプル」を計算する。
    const float left  = nextSample(*bus.left);
    const float right = nextSample(*bus.right);

    // I2S 用の 16bit PCM に変換して格納する。
    bus.buffer[2 * i]     = toPcm16(left);   // Left
    bus.buffer[2 * i + 1] = toPcm16(right);  // Right
  }
}


// ============================================================================
// 9. I2S へ書き込む
// ============================================================================

// 1本のバスに対して、PCM バッファを生成し、i2s_write で送る。
// 「途中までしか書けなかった」ケースも考えて、全部送るまでループする。
bool writeBus(BusState& bus) {
  // まず最新の PCM バッファを作る。
  fillBusBuffer(bus);

  const uint8_t* src = reinterpret_cast<const uint8_t*>(bus.buffer);
  const size_t total_bytes = sizeof(bus.buffer);
  size_t sent = 0;

  // total_bytes 分を書き切るまで繰り返す。
  while (sent < total_bytes) {
    size_t written = 0;

    const esp_err_t err = i2s_write(
      bus.port,
      src + sent,
      total_bytes - sent,
      &written,
      I2S_TIMEOUT_TICKS
    );

    // 書き込み API 自体が失敗した場合。
    if (err != ESP_OK) {
      logI2SError("i2s_write failed", bus.port, err);
      return false;
    }

    // タイムアウトなどで 1バイトも書けなかった場合。
    if (written == 0) {
      Serial.printf("[WARN] i2s_write timeout on port %d\n",
                    static_cast<int>(bus.port));
      return false;
    }

    sent += written;
  }

  return true;
}


// ============================================================================
// 10. テストパターン制御
// ============================================================================

// 指定した1チャンネルだけ有効化する。
// 自己診断モードでは、これで ch0→ch1→ch2→ch3 を順番に鳴らす。
void setOnlyOneChannel(uint8_t ch_index) {
  for (uint8_t i = 0; i < 4; ++i) {
    channels[i].enabled = (i == ch_index);
  }
}

// テストモードに応じて、どのチャンネルを鳴らすか更新する。
// この関数は loop() ごとに呼ばれる。
void updateTestPattern() {
  // 4ch 同時出力モードなら、全チャンネルを有効化するだけでよい。
  if (TEST_MODE == TestMode::AllChannelsDifferentFreq) {
    for (uint8_t i = 0; i < 4; ++i) {
      channels[i].enabled = true;
    }
    return;
  }

  // 以下は 1ch ずつ順番に鳴らすモード。
  const uint32_t now = millis();

  // 初回呼び出し時。
  if (last_step_ms == 0) {
    last_step_ms = now;
    active_channel = 0;
    setOnlyOneChannel(active_channel);
    Serial.println("[TEST] ch0 only");
    return;
  }

  // 一定時間経ったら次のチャンネルへ進める。
  if (now - last_step_ms >= STEP_MS) {
    last_step_ms = now;
    active_channel = (active_channel + 1) % 4;
    setOnlyOneChannel(active_channel);
    Serial.printf("[TEST] ch%u only\n", active_channel);
  }
}

}  // namespace audio4ch


// ============================================================================
// 11. Arduino setup()
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  using namespace audio4ch;

  Serial.println();
  Serial.println("==== 4ch actuator bring-up ====");

  // ここで注意:
  // M5StickS3 の EXT_5V をボード側から供給したい場合は、
  // M5Unified を使う構成であれば setup() 冒頭などで
  //   M5.Power.setExtOutput(true);
  // のような設定が必要になる場合がある。
  // ただし、今回は依存関係を増やさないため、このスケッチ単体には入れていない。

  // 2本の I2S バスを順に初期化する。
  const bool ok0 = setupI2S(bus0);
  const bool ok1 = setupI2S(bus1);

  // どちらかが失敗したら、その場で停止する。
  // 「片方だけ失敗したのに、何となく loop に入り続ける」より、
  // bring-up の段階ではこちらの方が問題を見つけやすい。
  if (!ok0 || !ok1) {
    Serial.println("[FATAL] I2S initialization failed.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("[INFO] Sequential self-test started.");
}


// ============================================================================
// 12. Arduino loop()
// ============================================================================

void loop() {
  using namespace audio4ch;

  // まず、今どのチャンネルを鳴らすべきかを更新する。
  updateTestPattern();

  // 2本の I2S バスへ順番に書き込む。
  // ここでは bus0 と bus1 を独立に扱う。
  const bool ok0 = writeBus(bus0);
  const bool ok1 = writeBus(bus1);

  // 失敗した場合は少し待つ。
  // 必要ならここで再初期化や詳細ログ追加もできるが、
  // まずは「異常が起きたことを観察する」ことを優先して簡素にしている。
  if (!ok0 || !ok1) {
    delay(10);
  }
}
