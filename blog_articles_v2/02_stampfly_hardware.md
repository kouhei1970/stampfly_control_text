# StampFlyハードウェア完全解説 - StampFly制御システム第2回

## この記事で学ぶこと（2分で読める概要）

第1回で学んだ物理的連鎖（PWM→ESC→モータ→プロペラ→推力）を実現する、M5StampFlyの具体的なハードウェア構成を詳しく解説します：

1. **ESP32-S3の制御能力**：なぜ240MHzデュアルコアが400Hz制御を可能にするのか
2. **センサ群の役割と特性**：BMI270（IMU）、BMP280（気圧）、VL53L3CX（距離）の統合
3. **パワーシステムの詳細**：716-17600kvモータとESCの選定理由と特性

**重要な発見**：各ハードウェアコンポーネントは、物理的連鎖の特定の役割を担うように最適化されており、その統合が安定飛行を実現します。

## 基礎知識の整理（初中級者向け詳細解説）

### マイクロドローンのハードウェア要件

マイクロドローンの制御システムには、以下の3つの基本要件があります：

1. **高速な計算能力**：400Hzのカスケード制御を実行
2. **精密なセンシング**：機体の状態を正確に把握
3. **効率的な動力系**：小型軽量で高応答性

M5StampFlyは、これらの要件を**36.8g**という超軽量な機体に凝縮しています。

### なぜESP32-S3なのか？

ESP32-S3は、ドローン制御に理想的な特性を持っています：

- **デュアルコア240MHz**：並列処理による高速制御
- **豊富なペリフェラル**：I2C、SPI、PWM出力を同時使用
- **FreeRTOS内蔵**：リアルタイムタスク管理
- **低消費電力**：限られたバッテリー容量を有効活用
- **標準通信機能**：WiFi、Bluetooth、ESP-NOWが標準搭載

特に重要なのは、**2つのコアを独立して使える**ことと、**通信機能が標準装備**されていることです。これにより、センサ処理と制御演算を並列実行し、400Hzの制御周期を確実に維持しながら、同時に無線通信も行えます。

### ESP32-S3の通信機能がもたらす革新

従来のドローン制御では、別途プロポ（送信機）と受信機が必要でした。しかし、ESP32-S3の標準通信機能により、**簡単にラジコンシステムを構築**できます：

#### **WiFi + ESP-NOWの威力**

```cpp
// ESP-NOWによる低遅延通信の実装例
void setup_espnow_communication() {
    // WiFiを初期化してESP-NOWを有効化
    WiFi.mode(WIFI_STA);
    esp_now_init();
    
    // ペアリング情報を登録（MACアドレスベース）
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, controller_mac, 6);
    esp_now_add_peer(&peer_info);
    
    ESP_LOGI(TAG, "ESP-NOW通信準備完了");
}
```

#### **通信機能の3つのメリット**

**1. コスト削減**：
- 従来：ドローン + プロポ + 受信機 = 高コスト
- ESP32-S3：本体のみで完結 = 大幅コスト削減

**2. 開発の簡素化**：
- 独自のコントローラーを簡単に作成可能
- スマートフォンアプリからの直接制御
- デバッグ情報のリアルタイム送信

**3. 拡張性**：
- **ESP-NOW**：最大20台のペア接続、ブロードキャスト通信で数百台対応（群制御対応）
- **WiFi**：インターネット経由での遠隔制御
- **Bluetooth**：近距離での安定通信

この通信機能により、M5StampFlyは**学習用プラットフォーム**として最適な選択となっています。プロポを別途購入することなく、すぐにドローン制御の実験を開始できます。

### センサ融合の重要性

ドローンの安定飛行には、複数のセンサデータを統合する必要があります：

**I2Cバス接続センサ（400kHz）**：

- **磁気センサ（BMM150）**：3軸地磁気検出（方位推定用）
- **気圧計（BMP280）**：気圧による高度推定
- **距離センサ（VL53L3CX×2）**：ToF方式による対地距離測定
- **電圧・電流モニタ（INA3221）**：バッテリー状態監視

**SPIバス接続センサ（最大10MHz）**：

- **IMU（BMI270）**：6軸慣性センサ（角速度と加速度）
- **オプティカルフロー（PMW3901MB）**：光学式の水平移動検出

これらのセンサは異なるインターフェースで接続され、それぞれの特性に最適化された通信速度で動作します。I2Cは配線が簡単で複数デバイスを接続でき、SPIは高速データ転送が必要なIMU（BMI270）とオプティカルフローに使用されています。

## 初心者のためのソフトウェア設計基礎：ハードウェアを扱う賢い方法

### なぜ「ハードウェア抽象化」が必要なのか？

ドローンのプログラミングを始めた初心者が最初に困るのは、「センサから値を読む」「モーターを回す」といった基本的な操作でも、複雑な設定コードが必要なことです。

**生のハードウェア操作の問題点**を、実際のコードで見てみましょう：

```cpp
// 悪い例：毎回複雑な設定が必要
void readIMU() {
    // SPI設定（毎回書く必要がある）
    spi_device_interface_config_t dev_config;
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 10000000;
    dev_config.spics_io_num = 46;
    // ... 20行以上の設定コード
    
    // やっとデータ読み取り
    uint8_t data[12];
    spi_device_transmit(spi_handle, &trans);
}
```

この問題を解決するのが**ハードウェア抽象化**です。

### ハードウェア抽象化とは？（初心者向け解説）

**簡単に言うと**：複雑なハードウェア操作を、簡単に使える「道具箱」にまとめることです。

**例え話**：
- **生のハードウェア操作** = 「毎回エンジンを分解して修理する」
- **ハードウェア抽象化** = 「車のアクセルペダルを踏むだけで加速する」

**抽象化後の使いやすさ**を、同じ処理のコードで比較してみましょう：

```cpp
// 良い例：簡単に使える
M5StampFlyHardware hardware;
auto imu_data = hardware.readIMU();  // たった1行！
```

### 抽象化の3つのメリット（初心者が実感できる利点）

#### 1. **理解しやすさ**
複雑な技術的詳細を隠すことで、「何をしたいか」に集中できます。以下のコード例で比較してみましょう：

```cpp
// 「IMUから値を読みたい」という意図が明確
auto data = imu.readData();

// vs 毎回SPIの設定を書くのは本質的でない
spi_device_interface_config_t config = {...};
```

#### 2. **間違いにくさ**
設定ミスやピンの間違いを防げます。具体的なコード例で見てみましょう：

```cpp
// ピン番号を間違える心配がない
motors.setSpeed(0, 1500);  // モーター0番を1500μsで制御

// vs 毎回ピン番号を書くと間違いやすい
ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
```

#### 3. **再利用しやすさ**
一度作った「道具箱」は、他のプロジェクトでも使えます。

### StampFlyならではのハードウェア設計の工夫

M5StampFlyのハードウェア構成には、実は深い設計思想があります：

#### **なぜBMI270はSPI接続なのか？**

**理由**：IMUは最も高頻度でデータを読み取る必要があるためです。

- **I2C（400kHz）**：1秒間に約400,000回通信可能
- **SPI（10MHz）**：1秒間に約10,000,000回通信可能

**400Hzの制御周期**では、IMUを2.5ms毎に読む必要があり、SPIの高速性が必要不可欠です。

#### **なぜ2つのLEDがあるのか？**

- **内蔵LED（G21）**：デバッグ時の状態表示
- **外部LED（G39）**：飛行中の機体識別

それぞれ異なる目的で最適化されています。

#### **なぜGrove端子が2つなのか？**

- **赤Grove（I2C）**：センサ拡張用
- **黒Grove（UART）**：通信・デバッグ用、S.BUS等の受信機接続も可能

センサ追加と通信を分けることで、拡張性を確保しています。

このように、M5StampFlyの各部品配置には「使いやすさ」と「性能」を両立する設計思想が込められています。

## 実装詳解：M5StampFlyHardwareクラス

これまでの設計思想を実際のC++コードで具体化します。以下のコードは、StampFlyの全ハードウェア構成を管理する統合クラスの実装です：

```cpp
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "M5STAMPFLY_HW";

// ハードウェア構成を管理する統合クラス
class M5StampFlyHardware {
public:
    // ハードウェアピン定義（M5StampFly実装準拠）
    struct PinConfig {
        // I2Cバス（BMM150, BMP280, VL53L3×2, INA3221）
        static constexpr int I2C_SDA = 3;   // GPIO3
        static constexpr int I2C_SCL = 4;   // GPIO4
        
        // SPIバス（BMI270, PMW3901MBオプティカルフロー）
        static constexpr int SPI_MOSI = 14;  // GPIO14
        static constexpr int SPI_MISO = 43;  // GPIO43 
        static constexpr int SPI_SCLK = 44;  // GPIO44
        static constexpr int SPI_CS_BMI270 = 46;  // GPIO46
        static constexpr int SPI_CS_FLOW = 12;    // GPIO12
        
        // モータPWM出力（実機StampFly準拠）
        static constexpr int MOTOR1_PIN = 6;   // GPIO6  (右前 CCW)
        static constexpr int MOTOR2_PIN = 5;   // GPIO5  (左前 CW)
        static constexpr int MOTOR3_PIN = 8;   // GPIO8  (左後 CCW)
        static constexpr int MOTOR4_PIN = 7;   // GPIO7  (右後 CW)
        
        // Grove端子
        static constexpr int GROVE_I2C_SDA = 13;  // GPIO13 (赤)
        static constexpr int GROVE_I2C_SCL = 15;  // GPIO15 (赤)
        static constexpr int GROVE_UART_TX = 1;   // GPIO1  (黒)
        static constexpr int GROVE_UART_RX = 2;   // GPIO2  (黒)
        
        // その他
        static constexpr int LED_PIN = 39;         // GPIO39 (WS2812 外部LED)
        static constexpr int STAMPS3_LED_PIN = 21; // GPIO21 (M5StampS3内蔵LED)
        static constexpr int BUZZER_PIN = 12;      // GPIO12
    };
    
    // システム状態
    struct SystemStatus {
        bool imu_ok = false;
        bool barometer_ok = false;
        bool motors_ok = false;
        bool battery_ok = false;
        float battery_voltage = 0.0f;
        float temperature = 0.0f;
        uint32_t error_count = 0;
    };
    
private:
    // センサオブジェクト（SPI/I2Cバス接続）
    class BMI270Sensor {
    public:
        static constexpr int SPI_CS_PIN = 46;  // GPIO46
        
        struct Data {
            float gyro_x, gyro_y, gyro_z;      // [rad/s]
            float accel_x, accel_y, accel_z;   // [m/s²]
            uint32_t timestamp;                 // [μs]
        };
        
        esp_err_t initialize() {
            ESP_LOGI(TAG, "BMI270 初期化開始 (SPI接続)");
            
            // SPIデバイス設定
            spi_device_interface_config_t dev_config = {
                .mode = 0,                    // SPI mode 0
                .clock_speed_hz = 10000000,   // 10MHz
                .spics_io_num = SPI_CS_PIN,   // CS pin
                .queue_size = 7,
                .flags = SPI_DEVICE_HALFDUPLEX
            };
            
            // SPIセンサ設定コマンド
            uint8_t config_data[] = {
                0x40, 0x2E,  // PWR_CONF: Normal mode
                0x42, 0x4C,  // ACC_CONF: 100Hz, Normal mode
                0x43, 0x0C,  // GYR_CONF: 100Hz, Normal mode
            };
            
            // TODO: 実際のSPI通信実装
            
            return ESP_OK;
        }
        
        Data readData() {
            Data data;
            // BMI270からの6軸データ読み取り（SPI経由）
            // 実際の実装では割り込みやDMAを使用
            
            // SPI読み取りトランザクション
            spi_transaction_t trans = {
                .length = 12 * 8,  // 12バイト（6軸×2バイト）
                .tx_buffer = nullptr,
                .rx_buffer = &data
            };
            
            // TODO: 実際のSPI通信実装
            
            return data;
        }
    };
    
    class BMM150Sensor {
    public:
        static constexpr uint8_t I2C_ADDR = 0x10;  // or 0x11, 0x12, 0x13
        
        struct Data {
            float mag_x, mag_y, mag_z;  // [μT]
            float heading;              // [rad]
        };
        
        esp_err_t initialize() {
            ESP_LOGI(TAG, "BMM150 磁気センサ初期化");
            return ESP_OK;
        }
    };
    
    class BMP280Sensor {
    public:
        static constexpr uint8_t I2C_ADDR = 0x76;  // or 0x77
        
        struct Data {
            float pressure;     // [Pa]
            float temperature;  // [℃]
            float altitude;     // [m]
        };
        
        esp_err_t initialize() {
            ESP_LOGI(TAG, "BMP280 初期化開始");
            // 気圧センサの初期化
            return ESP_OK;
        }
        
        Data readData() {
            Data data;
            // 気圧と温度から高度を計算
            return data;
        }
    };
    
    class MotorController {
    private:
        ledc_channel_t channels_[4];
        
    public:
        esp_err_t initialize() {
            ESP_LOGI(TAG, "モータコントローラ初期化開始");
            
            // LEDC（PWM）の設定
            ledc_timer_config_t timer_conf = {
                .speed_mode = LEDC_HIGH_SPEED_MODE,
                .duty_resolution = LEDC_TIMER_11_BIT,  // 11bit = 2048段階
                .timer_num = LEDC_TIMER_0,
                .freq_hz = 500,  // PWM周波数 500Hz
                .clk_cfg = LEDC_AUTO_CLK
            };
            ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
            
            // 4つのモータチャンネル設定
            int motor_pins[] = {
                PinConfig::MOTOR1_PIN,  // M1: 右前 CCW
                PinConfig::MOTOR2_PIN,  // M2: 左前 CW
                PinConfig::MOTOR3_PIN,  // M3: 左後 CCW
                PinConfig::MOTOR4_PIN   // M4: 右後 CW
            };
            
            for (int i = 0; i < 4; i++) {
                ledc_channel_config_t channel_conf = {
                    .gpio_num = motor_pins[i],
                    .speed_mode = LEDC_HIGH_SPEED_MODE,
                    .channel = (ledc_channel_t)i,
                    .timer_sel = LEDC_TIMER_0,
                    .duty = 0,
                    .hpoint = 0,
                    .flags = {.output_invert = 0}
                };
                ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
                channels_[i] = (ledc_channel_t)i;
            }
            
            return ESP_OK;
        }
        
        void setMotorPWM(int motor_id, uint16_t pwm_us) {
            // PWMマイクロ秒をデューティ比に変換
            // 500Hz = 2000μs周期、11bit解像度 = 2048段階
            uint32_t duty = (pwm_us * 2048) / 2000;
            duty = (duty > 2047) ? 2047 : duty;
            
            ESP_ERROR_CHECK(ledc_set_duty(
                LEDC_HIGH_SPEED_MODE, 
                channels_[motor_id], 
                duty
            ));
            ESP_ERROR_CHECK(ledc_update_duty(
                LEDC_HIGH_SPEED_MODE, 
                channels_[motor_id]
            ));
        }
    };
    
    // メンバ変数
    BMI270Sensor imu_;
    BMM150Sensor magnetometer_;
    BMP280Sensor barometer_;
    MotorController motors_;
    SystemStatus status_;
    
    // デュアルコア用のタスクハンドル
    TaskHandle_t sensor_task_handle_ = nullptr;
    TaskHandle_t control_task_handle_ = nullptr;
    
public:
    esp_err_t initialize() {
        ESP_LOGI(TAG, "M5StampFly ハードウェア初期化開始");
        
        // I2Cバスの初期化（BMM150, BMP280, VL53L3CX, INA3221用）
        i2c_config_t i2c_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = PinConfig::I2C_SDA,
            .scl_io_num = PinConfig::I2C_SCL,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {
                .clk_speed = 400000  // 400kHz
            }
        };
        ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
        ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
        
        // SPIバスの初期化（BMI270, PMW3901MB用）
        spi_bus_config_t spi_bus_config = {
            .mosi_io_num = PinConfig::SPI_MOSI,
            .miso_io_num = PinConfig::SPI_MISO,
            .sclk_io_num = PinConfig::SPI_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4096
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, 0));
        
        // 各コンポーネントの初期化
        status_.imu_ok = (imu_.initialize() == ESP_OK);
        status_.imu_ok &= (magnetometer_.initialize() == ESP_OK);  // 磁気センサ（I2C）
        status_.barometer_ok = (barometer_.initialize() == ESP_OK);
        status_.motors_ok = (motors_.initialize() == ESP_OK);
        
        // デュアルコアタスクの作成
        createDualCoreTasks();
        
        return (status_.imu_ok && status_.barometer_ok && status_.motors_ok) 
               ? ESP_OK : ESP_FAIL;
    }
    
    bool selfTest() {
        ESP_LOGI(TAG, "セルフテスト開始");
        
        // IMUの動作確認
        auto imu_data = imu_.readData();
        bool imu_test = (abs(imu_data.accel_z + 9.81f) < 1.0f);  // 重力加速度チェック
        
        // モータの動作確認（低速で短時間回転）
        bool motor_test = testMotors();
        
        // バッテリー電圧確認
        status_.battery_voltage = readBatteryVoltage();
        status_.battery_ok = (status_.battery_voltage > 3.3f);
        
        ESP_LOGI(TAG, "セルフテスト結果:");
        ESP_LOGI(TAG, "  IMU: %s", imu_test ? "OK" : "NG");
        ESP_LOGI(TAG, "  モータ: %s", motor_test ? "OK" : "NG");
        ESP_LOGI(TAG, "  バッテリー: %.2fV %s", 
                 status_.battery_voltage,
                 status_.battery_ok ? "OK" : "NG");
        
        return imu_test && motor_test && status_.battery_ok;
    }
    
    SystemStatus getStatus() const {
        return status_;
    }
    
private:
    void createDualCoreTasks() {
        // Core 0: センサ読み取りタスク（1kHz）
        xTaskCreatePinnedToCore(
            sensorTask,           // タスク関数
            "SensorTask",         // タスク名
            4096,                 // スタックサイズ
            this,                 // パラメータ
            24,                   // 優先度（高）
            &sensor_task_handle_, // タスクハンドル
            0                     // Core 0
        );
        
        // Core 1: 制御演算タスク（400Hz）
        xTaskCreatePinnedToCore(
            controlTask,
            "ControlTask",
            8192,
            this,
            23,                   // 優先度（センサより少し低い）
            &control_task_handle_,
            1                     // Core 1
        );
    }
    
    static void sensorTask(void* parameter) {
        M5StampFlyHardware* hw = static_cast<M5StampFlyHardware*>(parameter);
        TickType_t last_wake_time = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(1);  // 1ms = 1kHz
        
        ESP_LOGI(TAG, "センサタスク開始 (Core 0)");
        
        while (1) {
            // IMUデータ読み取り
            auto imu_data = hw->imu_.readData();
            
            // データをキューで制御タスクに送信
            // （実装省略）
            
            // 正確な周期で実行
            vTaskDelayUntil(&last_wake_time, period);
        }
    }
    
    static void controlTask(void* parameter) {
        M5StampFlyHardware* hw = static_cast<M5StampFlyHardware*>(parameter);
        TickType_t last_wake_time = xTaskGetTickCount();
        const TickType_t period = pdMS_TO_TICKS(2.5);  // 2.5ms = 400Hz
        
        ESP_LOGI(TAG, "制御タスク開始 (Core 1)");
        
        while (1) {
            // センサデータを受信
            // カスケード制御演算
            // モータPWM出力
            
            vTaskDelayUntil(&last_wake_time, period);
        }
    }
    
    bool testMotors() {
        // 安全のため低速で短時間だけ回転
        const uint16_t test_pwm = 1100;  // わずかに回転する程度
        const int test_duration_ms = 200;
        
        for (int i = 0; i < 4; i++) {
            motors_.setMotorPWM(i, test_pwm);
            vTaskDelay(pdMS_TO_TICKS(test_duration_ms));
            motors_.setMotorPWM(i, 1000);  // 停止
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        return true;  // 実際は電流センサ等で確認
    }
    
    float readBatteryVoltage() {
        // ADCから電圧読み取り（分圧回路考慮）
        // 実装省略
        return 3.8f;  // ダミー値
    }
};
```

このクラス設計により、ハードウェアの複雑さを隠蔽しながら、必要な機能を提供しています。

## 実装詳解（コード中心）

### デュアルコアの活用戦略

ESP32-S3の2つのコアを効果的に使い分けることが、高速制御の鍵です。ただし、ESP-IDFの重要な制約があります：

**Core 0（WiFi/ESP-NOW + 軽量処理）**：

- WiFi関連タスク（ESP-IDFがデフォルトで配置）
- ESP-NOW通信処理
- システムモニタリング
- 低優先度のセンサ処理

**Core 1（制御演算専用）**：

- 400Hzでカスケード制御実行
- 高速センサ読み取り（IMU等）
- モータミキシング計算
- PWM信号出力

**重要**：ESP-IDFではWiFi機能（ESP-NOWを含む）がCore 0で動作するため、制御の主要部分はCore 1に集約する設計が必要です。これにより、通信による制御への影響を最小限に抑えられます。

## よくある問題と対処法

### 初学者がつまずきやすいポイント

**1. 「I2C通信エラーが頻発する」**

**対処法**：プルアップ抵抗の確認

- 内蔵プルアップだけでは不足する場合がある
- 外付けで4.7kΩの抵抗を追加
- 配線長を最小限に

**2. 「センサ値がおかしい」**

**対処法**：電源ノイズの除去

```cpp
// ソフトウェアフィルタの実装
class LowPassFilter {
private:
    float alpha_ = 0.1f;  // フィルタ係数
    float filtered_value_ = 0.0f;
    
public:
    float update(float raw_value) {
        filtered_value_ = alpha_ * raw_value + (1.0f - alpha_) * filtered_value_;
        return filtered_value_;
    }
};
```

**3. 「モータが振動する」**

**対処法**：IMUの振動対策

- 防振マウントの使用
- ソフトマウントでの固定
- ノッチフィルタの実装

### デバッグテクニック

```cpp
// リアルタイムデバッグ用マクロ
#define DEBUG_PRINT_RATE(msg, rate_hz) \
    do { \
        static uint32_t last_print = 0; \
        uint32_t now = esp_timer_get_time() / 1000; \
        if (now - last_print > (1000 / rate_hz)) { \
            ESP_LOGI(TAG, msg); \
            last_print = now; \
        } \
    } while(0)

// 使用例：1Hzでステータス表示
DEBUG_PRINT_RATE("IMU OK", 1);
```

## 発展的内容（上級者向け）

### ESP-NOWとコア配分の詳細

ESP-IDFでは、WiFi関連機能（ESP-NOWを含む）は**デフォルトでCore 0に割り当てられます**。これは重要な制約です：

```cpp
// ESP-IDFのWiFiタスク配置
// WiFi関連タスク（ESP-NOW含む）はCore 0で実行される
// これを考慮したタスク配分が必要

class M5StampFlyHardwareAdvanced : public M5StampFlyHardware {
private:
    void createOptimizedTasks() {
        // Core 0: WiFi/ESP-NOW + 軽量タスク
        xTaskCreatePinnedToCore(
            espNowAndMonitorTask,
            "ESPNowMonitor",
            4096,
            this,
            20,  // WiFiより低優先度
            &espnow_task_handle_,
            0    // Core 0（WiFiと同じ）
        );
        
        // Core 1: 制御演算 + センサ処理（最適化版）
        xTaskCreatePinnedToCore(
            controlAndSensorTask,
            "ControlSensor", 
            8192,
            this,
            24,  // 最高優先度
            &control_task_handle_,
            1    // Core 1（制御専用）
        );
    }
    
    static void espNowAndMonitorTask(void* parameter) {
        // ESP-NOW通信とシステムモニタリング
        // WiFiタスクの合間に実行
        while (1) {
            // テレメトリ送信（10Hz程度）
            sendTelemetry();
            
            // システム状態監視
            monitorSystemHealth();
            
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    
    static void controlAndSensorTask(void* parameter) {
        // Core 1で制御とセンサを統合処理
        TickType_t last_wake_time = xTaskGetTickCount();
        
        while (1) {
            // センサ読み取り（高速）
            readSensorsOptimized();
            
            // 制御演算（400Hz）
            executeControl();
            
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2.5));
        }
    }
};
```

この配分により、WiFi通信による制御への影響を最小限に抑えています。

### 代替センサの選定

**IMUの選択肢**：

- **BMI270**：省電力、高精度（M5StampFly採用）
- **MPU6050**：安価、実績豊富
- **ICM-42688-P**：最新、超高精度

選定基準：

- ノイズレベル
- サンプリングレート
- 消費電力
- 温度特性

### リアルタイムOS活用の深掘り

FreeRTOSの高度な機能活用：

```cpp
// タスク実行時間の計測
void measureTaskPerformance() {
    TaskStatus_t task_status[10];
    uint32_t total_runtime;
    
    int task_count = uxTaskGetSystemState(task_status, 10, &total_runtime);
    
    for (int i = 0; i < task_count; i++) {
        uint32_t runtime_percent = (task_status[i].ulRunTimeCounter * 100) / total_runtime;
        ESP_LOGI(TAG, "タスク %s: CPU使用率 %d%%", 
                 task_status[i].pcTaskName, 
                 runtime_percent);
    }
}
```

## まとめと次回予告

### 今回学んだ重要なポイント

1. **デュアルコア活用**：センサ処理と制御演算の並列実行
2. **ハードウェア抽象化**：保守性とテスタビリティの向上
3. **リアルタイム性の確保**：FreeRTOSによる確定的なタスク実行
4. **統合的な設計**：各コンポーネントの協調動作

### なぜこれが重要なのか

ハードウェアの理解は、制御システム設計の基盤です。各コンポーネントの特性と制約を知ることで、より効果的な制御アルゴリズムを実装できます。

### 次回予告：第3回「モータ・ESC・プロペラシステム」

次回は、物理的連鎖の中核となるパワーシステムを詳しく解説します：

- **716-17600kvモータ**の電気的・機械的特性
- **ESCの動作原理**：PWM信号から三相交流への変換
- **プロペラ空力**：推力とトルクの発生メカニズム
- **システム統合**：`MotorESCSystem`クラスの実装

**予習のポイント**：ブラシレスモータの基本原理と、PWM信号によるスピード制御について調べておくと、次回の理解が深まります。

---

**シリーズ**: 基礎・ハードウェア編  
**対象読者**: 初級〜中級  
**推定読了時間**: 10分  
**メインクラス**: `M5StampFlyHardware`  
**重点ポイント**: デュアルコア活用とハードウェア抽象化  
**物理的連鎖**: PWM → ESC → モータ → プロペラ → 推力 → 機体運動

**関連記事**: 
- [第1回: ドローンの世界への扉](01_drone_introduction.md)
- [第3回: モータ・ESC・プロペラシステム](03_motor_esc_propeller.md)

**質問・感想をお待ちしています**：実装で困ったことや、さらに知りたい内容があれば、ぜひコメントでお聞かせください。

---

## 免責事項

本記事の内容は教育・学習目的で提供されています。ドローンの製作・飛行に関しては以下の点にご注意ください：

- **法規制の遵守**：航空法、電波法等の関連法規を必ず確認し、遵守してください
- **安全性の確保**：機体の整備、飛行環境の安全確認は製作者・操縦者の責任です
- **技術的内容**：記事中のコードや設計は例示であり、実際の使用時は十分な検証が必要です
- **損害の免責**：本記事の内容に起因する損害について、著者は責任を負いかねます

安全第一でドローン技術を学び、楽しんでください。