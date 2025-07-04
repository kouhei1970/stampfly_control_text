# ドローンの世界への扉 - StampFly制御システム第1回

## この記事で学ぶこと（2分で読める概要）

この記事では、ドローン制御の本質を理解するための第一歩として、以下の3つのポイントを学びます：

1. **ドローンが飛ぶ仕組み**：PWM信号からプロペラの推力発生まで、物理的な連鎖プロセス
2. **M5StampFlyの特徴**：なぜ学習に最適なプラットフォームなのか
3. **学習ロードマップ**：30回シリーズで制御理論から実装まで体系的に習得

**重要な発見**：ドローンは「不安定な系を制御技術で安定化した飛行機械」であり、この制御の連鎖を理解することが全ての基盤となります。

## この連載シリーズの特徴

### 理論と実装の一体化アプローチ

この「StampFly制御システム」シリーズでは、**具体的なC++コードを通して概念を学ぶ**ことを重視しています。抽象的な説明だけでなく、実際に動作するコードを見ることで：

- **理解の曖昧さを排除**：「なんとなく分かった」ではなく、確実な理解を促進
- **実装への橋渡し**：理論から実際のプログラミングまでをシームレスに学習
- **デバッグ力の向上**：実際のコードを読むことで、問題解決能力を養成

### コード掲載の方針

記事中にコードが登場する際は、必ず以下の情報を併記します：

1. **なぜそのコードが必要なのか**（掲載の理由）
2. **何を実現するコードなのか**（目的と機能）
3. **どのように読み解くべきか**（理解のポイント）

例えば、「センサデータの読み取り」について説明する際：

```cpp
// バッテリー電圧を監視する実装例
float voltage = battery_monitor.getVoltage();
if (voltage < 3.3f) {
    emergency_landing();  // 緊急着陸
}
```

このように、コードは説明を補強し、具体的な実装イメージを提供する役割を担います。

## 基礎知識の整理（初中級者向け詳細解説）

### ドローンとは何か？

ドローン（UAV: Unmanned Aerial Vehicle）は、人が搭乗せずに飛行する航空機の総称です。しかし、技術的な観点から見ると、ドローンの本質は**「制御システムによって安定化された不安定な機械システム」**にあります。

従来の飛行機は、設計上ある程度の安定性を持っています。一方、マルチコプタ型ドローンは、**本質的に不安定**です。パイロットや制御システムが常に調整し続けなければ、即座に墜落してしまいます。

### なぜマルチコプタは不安定なのか？

これを理解するために、手のひらの上で鉛筆を立てることを考えてみましょう：

- **鉛筆を手のひらで立てる**：完全な静的不安定状態で、微小な傾きが増幅して倒れます。手を絶えず動かして調整することで、かろうじて立った状態を維持できます
- **マルチコプタ**：同じように常に不安定で、制御システムが**毎秒400回**の調整（手の動きに相当）を行うことで飛行を維持しています

この頻繁な調整こそが、ドローン制御技術の核心なのです。

### 物理的連鎖：PWMから飛行まで

ドローンが飛ぶまでのプロセスは、以下の物理的連鎖で説明できます：

```
PWM信号 → ESC制御 → モータ回転 → プロペラ推力 → 機体運動 → センサ検出 → 制御計算 → PWM信号
    ↑                                                                                    ↓
    └─────────────────── フィードバック制御ループ ──────────────────────┘
```

この連鎖の各要素を理解することが、ドローン制御をマスターする鍵となります。

1. **PWM信号**：マイコンから出力される制御信号（通常1000-2000μs）
2. **ESC制御**：電子スピードコントローラがモータを駆動
3. **モータ回転**：ブラシレスモータが回転力を生成
4. **プロペラ推力**：回転するプロペラが空気を加速して推力を生成
5. **機体運動**：推力が機体に作用して6自由度運動を生成
6. **センサ検出**：IMU等のセンサが機体の状態を測定
7. **制御計算**：マイコンが目標値と現在値の差を計算
8. **PWM信号**：計算結果に基づいて新しいPWM信号を出力

この循環が**400Hz**（0.0025秒間隔）で実行されることで、人間には不可能な高速制御が実現されています。

## M5StampFlyの設計思想

### なぜM5StampFlyなのか？

M5StampFlyは、ドローン制御学習に最適化された教育プラットフォームとして設計されています。その特徴を詳しく見てみましょう。

```cpp
// M5StampFlyの基本仕様を表すクラス
class M5StampFlySpecs {
public:
    // ハードウェア仕様
    static constexpr struct {
        const char* mcu = "ESP32-S3";           // デュアルコア 240MHz
        const char* imu = "BMI270";             // 6軸IMU
        const char* barometer = "BMP280";       // 気圧センサ
        const char* motor = "716-17600kv";      // 高速ブラシレスモータ
        const char* battery = "300mAh 1S HV";   // 高電圧リチウムポリマー
        float weight = 36.8f;                   // 総重量 [g]
        float frame_size = 81.5f;               // フレームサイズ [mm]
    } hardware;
    
    // 制御性能仕様
    static constexpr struct {
        int control_frequency = 400;            // 制御周波数 [Hz]
        int sensor_frequency = 1000;            // センサ更新頻度 [Hz]
        float max_tilt_angle = 30.0f;          // 最大傾斜角 [度]
        float max_climb_rate = 2.0f;           // 最大上昇率 [m/s]
        int communication_latency = 10;        // 通信遅延 [ms]
    } performance;
    
    // 安全性仕様
    static constexpr struct {
        float min_battery_voltage = 3.4f;      // 最低バッテリー電圧 [V]
        float max_motor_temperature = 80.0f;   // 最高モータ温度 [℃]
        bool propeller_guards = false;         // プロペラガード（別売）
        const char* flight_area = "indoor";    // 推奨飛行エリア
    } safety;
};
```

### 教育プラットフォームとしての優位性

**1. 学習しやすいサイズ**
- **手のひらサイズ**：室内での安全な実験が可能
- **軽量設計**：事故時の被害を最小限に抑制
- **可視性**：動作を目視で確認しやすい

**2. 高性能な制御システム**
- **ESP32-S3**：本格的な制御アルゴリズムを実装可能
- **高精度センサ**：プロ仕様のIMUとセンサフュージョン
- **リアルタイム制御**：FreeRTOSによる確定的なタスクスケジューリング

**3. オープンな開発環境**
- **ESP-IDF**：産業レベルの開発フレームワーク
- **完全オープンソース**：全ての設計とコードが公開
- **コミュニティサポート**：活発な技術コミュニティ

## 実装詳解（コード中心）

### 基本的な制御ループの実装

M5StampFlyの制御システムは、**カスケード制御**と呼ばれる二重ループ構造で実装されています。これは実際のドローンで広く採用されている制御方式です：

```cpp
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "STAMPFLY_INTRO";

// ドローンの基本状態を表現するクラス
class DroneBasicState {
public:
    // 姿勢（外側ループで使用）
    struct Attitude {
        float roll = 0.0f;      // ロール角 [ラジアン]
        float pitch = 0.0f;     // ピッチ角 [ラジアン]
        float yaw = 0.0f;       // ヨー角 [ラジアン]
    } attitude;
    
    // 角速度（内側ループで使用）
    struct AngularRate {
        float roll_rate = 0.0f;   // ロール角速度 [rad/s]
        float pitch_rate = 0.0f;  // ピッチ角速度 [rad/s]
        float yaw_rate = 0.0f;    // ヨー角速度 [rad/s]
    } angular_rate;
    
    // モータコマンド
    struct MotorCommands {
        uint16_t motor1 = 1000; // 前右モータ [μs]
        uint16_t motor2 = 1000; // 前左モータ [μs]
        uint16_t motor3 = 1000; // 後左モータ [μs]
        uint16_t motor4 = 1000; // 後右モータ [μs]
    } motors;
    
    // 状態の表示（デバッグ用）
    void printState() const {
        ESP_LOGI(TAG, "姿勢: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°",
                 attitude.roll * 180.0f / M_PI,
                 attitude.pitch * 180.0f / M_PI,
                 attitude.yaw * 180.0f / M_PI);
        ESP_LOGI(TAG, "角速度: Roll=%.2f°/s, Pitch=%.2f°/s, Yaw=%.2f°/s",
                 angular_rate.roll_rate * 180.0f / M_PI,
                 angular_rate.pitch_rate * 180.0f / M_PI,
                 angular_rate.yaw_rate * 180.0f / M_PI);
        ESP_LOGI(TAG, "モータ: M1=%d, M2=%d, M3=%d, M4=%d",
                 motors.motor1, motors.motor2, motors.motor3, motors.motor4);
    }
};

// カスケード制御による基本的な制御ループ
class BasicFlightController {
private:
    DroneBasicState current_state_;
    DroneBasicState target_state_;
    
    // カスケード制御のゲイン
    float kp_attitude_ = 5.0f;    // 姿勢制御のPゲイン（外側ループ）
    float kp_rate_ = 0.2f;        // 角速度制御のPゲイン（内側ループ）
    
public:
    // 制御ループ（400Hz で実行）
    void controlLoop() {
        // 1. センサからの状態読み取り（実際の実装は後の記事で詳解）
        readSensors();
        
        // 2. カスケード制御：外側ループ（姿勢制御）
        // 目標姿勢角から目標角速度を計算
        float target_roll_rate = kp_attitude_ * 
            (target_state_.attitude.roll - current_state_.attitude.roll);
        float target_pitch_rate = kp_attitude_ * 
            (target_state_.attitude.pitch - current_state_.attitude.pitch);
        float target_yaw_rate = kp_attitude_ * 
            (target_state_.attitude.yaw - current_state_.attitude.yaw);
        
        // 3. カスケード制御：内側ループ（角速度制御）
        // 目標角速度と現在の角速度の差分から制御量を計算
        float roll_output = kp_rate_ * 
            (target_roll_rate - current_state_.angular_rate.roll_rate);
        float pitch_output = kp_rate_ * 
            (target_pitch_rate - current_state_.angular_rate.pitch_rate);
        float yaw_output = kp_rate_ * 
            (target_yaw_rate - current_state_.angular_rate.yaw_rate);
        
        // 4. モータミキシング（X配置の場合）
        uint16_t base_throttle = 1200; // ベースとなるスロットル値
        
        current_state_.motors.motor1 = base_throttle + roll_output + pitch_output - yaw_output;
        current_state_.motors.motor2 = base_throttle - roll_output + pitch_output + yaw_output;
        current_state_.motors.motor3 = base_throttle - roll_output - pitch_output - yaw_output;
        current_state_.motors.motor4 = base_throttle + roll_output - pitch_output + yaw_output;
        
        // 5. 安全範囲内に制限
        constrainMotorOutputs();
        
        // 6. モータへの出力（実際のPWM出力は後の記事で詳解）
        outputToMotors();
    }
    
private:
    void readSensors() {
        // センサ読み取りの詳細実装は第4回で解説
        ESP_LOGD(TAG, "センサデータを読み取り中...");
    }
    
    void constrainMotorOutputs() {
        // モータ出力を安全範囲（1000-2000μs）に制限
        current_state_.motors.motor1 = std::clamp(current_state_.motors.motor1, 
                                                  (uint16_t)1000, (uint16_t)2000);
        current_state_.motors.motor2 = std::clamp(current_state_.motors.motor2, 
                                                  (uint16_t)1000, (uint16_t)2000);
        current_state_.motors.motor3 = std::clamp(current_state_.motors.motor3, 
                                                  (uint16_t)1000, (uint16_t)2000);
        current_state_.motors.motor4 = std::clamp(current_state_.motors.motor4, 
                                                  (uint16_t)1000, (uint16_t)2000);
    }
    
    void outputToMotors() {
        // PWM出力の詳細実装は第3回で解説
        ESP_LOGD(TAG, "モータ出力: %d, %d, %d, %d",
                current_state_.motors.motor1, current_state_.motors.motor2,
                current_state_.motors.motor3, current_state_.motors.motor4);
    }
};

// FreeRTOSタスクとしての実装例
void flight_control_task(void* parameter) {
    BasicFlightController controller;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(2.5); // 2.5ms = 400Hz
    
    ESP_LOGI(TAG, "飛行制御タスクを開始します");
    
    while (1) {
        // 制御ループ実行
        controller.controlLoop();
        
        // 次回実行まで待機（正確な周期で実行）
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}
```

このコードは、実際のドローン制御で使われる**カスケード制御**の基本構造を示しています。

**なぜカスケード制御が必要なのか？**

1. **安定性の向上**：角速度制御（内側ループ）が高速で応答し、外乱を素早く抑制
2. **調整の容易さ**：各ループを独立して調整でき、パラメータ設定が簡単
3. **実用性**：ほぼすべての実用的なドローンがこの構造を採用

**航空機制御システムとの類似性**

このカスケード制御は、有人航空機の制御システムと同じ考え方です：

- **内側ループ（角速度制御）**：**SAS（Stability Augmentation System：安定性増大システム）**として機能
  - 機体の揺れや振動を抑制
  - パイロット（または外側ループ）の操作を安定化
  - 高速応答により外乱の影響を最小化

- **外側ループ（姿勢制御）**：**CAS（Control Augmentation System：操縦性増大システム）**として機能
  - 目標姿勢への追従を実現
  - 操縦入力に対する応答性を向上
  - 飛行特性を改善し、扱いやすくする

直接姿勢制御だけでは、慣性の影響で振動しやすく、安定飛行は困難です。カスケード制御により、初心者でも比較的簡単にパラメータ調整が可能になります。

## よくある問題と対処法

### 初学者がつまずきやすいポイント

**1. 「制御が複雑すぎて理解できない」**

**対処法**：物理的連鎖を一つずつ理解する
- まずはPWM信号がモータを回すことを確認
- 次にプロペラが推力を生む様子を観察
- 最後に制御アルゴリズムを学習

**例**：まずは角速度制御（内側ループ）のゲインから調整
```cpp
// ステップ1：角速度制御のゲイン調整
void tune_rate_controller() {
    float kp_rate = 0.1f; // 小さい値から開始
    // 角速度応答を確認しながら徐々に増加
    
    // ステップ2：姿勢制御のゲイン調整
    float kp_attitude = 2.0f; // 角速度制御が安定してから調整
}
```

**2. 「数学が難しくて分からない」**

**対処法**：直感的理解から始める

- 三角関数は「傾き」を表現する道具
- 微分は「変化の速さ」
- 積分は「累積した量」

**3. 「コードが動かない」**

**対処法**：段階的デバッグ
```cpp
// デバッグ用のログ出力
void debug_system_state() {
    ESP_LOGI(TAG, "=== システム状態 ===");
    ESP_LOGI(TAG, "CPU使用率: %d%%", get_cpu_usage());
    ESP_LOGI(TAG, "メモリ使用量: %d bytes", get_memory_usage());
    ESP_LOGI(TAG, "制御周期: %.2f Hz", get_actual_control_frequency());
}
```

## 発展的内容（上級者向け）

### 現代的な制御手法への発展

基本的なPID制御から、より高度な制御手法への発展可能性：

**1. モデル予測制御（MPC）**：未来の挙動を予測して最適制御
```cpp
class ModelPredictiveController {
private:
    static constexpr int HORIZON = 10;  // 予測ホライズン
    float state_prediction_[HORIZON];
    
public:
    float computeOptimalControl(const DroneState& current_state) {
        // 未来の状態を予測し、制約を満たしながら最適化
        // 二次計画問題を解いて最適制御入力を計算
        return solveQP(current_state, state_prediction_);
    }
};
```

**2. 機械学習ベース制御**：TensorFlow Liteを活用
```cpp
class MLController {
private:
    TensorFlowLiteModel model_;
    
public:
    MotorCommands predict(const SensorData& sensors) {
        // ニューラルネットワークによる制御出力予測
        return model_.inference(sensors);
    }
};
```

**3. 群制御**：複数機体の協調制御
```cpp
class SwarmController {
public:
    void coordinateWithNeighbors(const std::vector<DroneState>& neighbors) {
        // フロッキングアルゴリズムによる群制御
    }
};
```

### 研究レベルの話題

- **非線形制御理論**：より正確な制御モデル
- **ロバスト制御**：外乱に対する頑健性
- **最適制御**：エネルギー効率の最適化
- **故障診断**：センサ・アクチュエータ故障の検出

## まとめと次回予告

### 今回学んだ重要なポイント

1. **ドローンの本質**：制御システムによって安定化された不安定なシステム
2. **物理的連鎖**：PWM→ESC→モータ→プロペラ→推力→運動→センサ→制御計算の循環
3. **M5StampFly**：学習に最適化された教育プラットフォーム
4. **制御の基本**：400Hzの高速フィードバックループによる安定化

### なぜこれが重要なのか

この物理的連鎖の理解は、ドローン制御のすべての基盤となります。どんなに高度な制御アルゴリズムも、最終的には「PWM信号でモータを回し、プロペラで推力を生成する」という物理的プロセスに帰結するからです。

### 次回予告：第2回「StampFlyハードウェア完全解説」

次回は、この物理的連鎖を実現するM5StampFlyのハードウェアを詳しく解説します：

- **ESP32-S3**：なぜ高速制御が可能なのか？
- **BMI270 IMU**：どのように姿勢を検出するのか？
- **716-17600kvモータ**：なぜこの仕様が選ばれたのか？
- **ハードウェア抽象化**：`M5StampFlyHardware`クラスの設計思想

**予習のポイント**：ESP32-S3のデュアルコア特性と、FreeRTOSタスクスケジューリングの基本概念を調べておくと、次回の理解が深まります。

**質問・感想をお待ちしています**：この記事で疑問に思った点や、実際に試してみた結果など、ぜひコメントでお聞かせください。読者の皆様からのフィードバックが、より良い解説の源となります。

---

**シリーズ**: 基礎・ハードウェア編  
**対象読者**: 全レベル  
**推定読了時間**: 8分  
**メインクラス**: なし（概念解説）  
**重点ポイント**: PWM→ESC→モータ→プロペラ→推力→機体運動の物理的連鎖  
**物理的連鎖**: PWM → ESC → モータ → プロペラ → 推力 → 機体運動

**関連記事**: [第2回: StampFlyハードウェア完全解説](02_stampfly_hardware.md)

**参考資料**:
- [StampFlyで学ぶマルチコプタ制御](https://www.docswell.com/s/Kouhei_Ito/K38V1P-2024-02-10-094123)
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343)

---

## 免責事項

本記事の内容は教育・学習目的で提供されています。ドローンの製作・飛行に関しては以下の点にご注意ください：

- **法規制の遵守**：航空法、電波法等の関連法規を必ず確認し、遵守してください
- **安全性の確保**：機体の整備、飛行環境の安全確認は製作者・操縦者の責任です
- **技術的内容**：記事中のコードや設計は例示であり、実際の使用時は十分な検証が必要です
- **損害の免責**：本記事の内容に起因する損害について、著者は責任を負いかねます

安全第一でドローン技術を学び、楽しんでください。
