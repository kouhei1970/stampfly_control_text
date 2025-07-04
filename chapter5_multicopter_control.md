# 第5章 マルチコプタ制御入門 - StampFly制御システム入門

## はじめに

前章まで、ドローンの物理モデルと制御理論の基礎を学んできました。本章では、これらの知識を統合し、M5StampFlyで実際に動作するマルチコプタ制御システムを実装します。10コマ分の内容として、センサフュージョン、姿勢制御、位置制御、そして実際のコード実装まで詳しく解説します。

## 5.1 M5StampFlyのハードウェア構成

### 5.1.1 システム概要

M5StampFlyは以下のコンポーネントで構成されています：

```c
// M5StampFlyの主要コンポーネント (ESP-IDF)
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/mcpwm.h"
#include "esp_now.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ハードウェア抽象化クラス
class M5StampFlyHardware {
private:
    // センサハンドル
    i2c_port_t i2c_port_;        // BMI270, BMM150, BMP280用
    spi_device_handle_t spi_handle_; // PMW3901MB, VL53L3CX用
    
    // モータ制御
    mcpwm_unit_t mcpwm_unit_;    // 716-17600kv モータ制御用
    
    // ESP-NOW通信
    esp_now_peer_info_t peer_info_;
    std::array<uint8_t, 6> controller_mac_;   // コントローラMACアドレス
    
    // FreeRTOSタスクハンドル
    TaskHandle_t sensor_task_;
    TaskHandle_t control_task_;
    TaskHandle_t comm_task_;
    
    // セマフォ・キュー
    SemaphoreHandle_t sensor_mutex_;
    QueueHandle_t control_queue_;
    
public:
    explicit M5StampFlyHardware();
    ~M5StampFlyHardware();
    
    // 初期化
    bool initialize();
    void shutdown();
    
    // アクセサ
    i2c_port_t getI2CPort() const noexcept { return i2c_port_; }
    spi_device_handle_t getSPIHandle() const noexcept { return spi_handle_; }
    mcpwm_unit_t getMCPWMUnit() const noexcept { return mcpwm_unit_; }
    
    // タスク管理
    bool createTasks();
    void destroyTasks();
    
    // 同期プリミティブ
    SemaphoreHandle_t getSensorMutex() const noexcept { return sensor_mutex_; }
    QueueHandle_t getControlQueue() const noexcept { return control_queue_; }
};
```

### 5.1.2 センサ仕様

BMI270の主な仕様：

```cpp
// IMUセンサの設定
class BMI270Config {
public:
    // 加速度計
    static constexpr float ACCEL_RANGE = 8.0;      // ±8g
    static constexpr float ACCEL_SCALE = 32768.0 / ACCEL_RANGE;
    static constexpr int ACCEL_RATE = 1000;        // Hz
    
    // ジャイロスコープ
    static constexpr float GYRO_RANGE = 2000.0;    // ±2000 deg/s
    static constexpr float GYRO_SCALE = 32768.0 / GYRO_RANGE;
    static constexpr int GYRO_RATE = 8000;         // Hz
    
    // ローパスフィルタ
    static constexpr int DLPF_BANDWIDTH = 20;      // Hz
};
```

### 5.1.3 モータとESC

```cpp
// モータ制御設定
class MotorConfig {
public:
    static constexpr int PWM_FREQUENCY = 400;      // Hz
    static constexpr int PWM_RESOLUTION = 12;      // bits (0-4095)
    static constexpr int MIN_THROTTLE = 1000;      // μs
    static constexpr int MAX_THROTTLE = 2000;      // μs
    static constexpr int KV_RATING = 17600;        // モータKV値
    
    // モータ配置（X配置）
    enum MotorIndex {
        FRONT_LEFT = 0,   // CW
        FRONT_RIGHT = 1,  // CCW
        REAR_RIGHT = 2,   // CW
        REAR_LEFT = 3     // CCW
    };
    
    // 機体仕様
    static constexpr float FRAME_SIZE = 81.5;      // mm
    static constexpr float WEIGHT = 36.8;          // g
};
```

### 5.1.4 追加センサの仕様

```cpp
// 磁力計設定
class BMM150Config {
public:
    static constexpr float MEASUREMENT_RANGE = 1300.0;  // μT
    static constexpr int DATA_RATE = 10;                // Hz
    static constexpr int POWER_MODE = BMM150_NORMAL;
};

// 気圧センサ設定
class BMP280Config {
public:
    static constexpr float PRESSURE_RANGE = 1100.0;     // hPa
    static constexpr float ALTITUDE_RESOLUTION = 0.12;  // m
    static constexpr int SAMPLING_RATE = 25;            // Hz
    static constexpr int OVERSAMPLING = 16;             // x16
};

// 距離センサ設定
class VL53L3CXConfig {
public:
    static constexpr float MAX_RANGE = 3000.0;          // mm
    static constexpr float ACCURACY = 20.0;             // mm
    static constexpr int MEASUREMENT_RATE = 30;         // Hz
    static constexpr int SENSOR_COUNT = 2;              // 下向き×2
};

// オプティカルフローセンサ設定
class PMW3901MBConfig {
public:
    static constexpr int RESOLUTION = 3600;             // counts/m
    static constexpr int UPDATE_RATE = 100;             // Hz
    static constexpr float MIN_DISTANCE = 80.0;         // mm
    static constexpr float MAX_DISTANCE = 3000.0;       // mm
};
```

## 5.2 制御システムアーキテクチャ

### 5.2.1 階層的制御構造

```cpp
// 制御システムの階層構造
class FlightController {
private:
    // センサ処理層
    IMUProcessor imu_processor;
    StateEstimator state_estimator;
    
    // 制御層（内側から外側へ）
    RateController rate_controller;        // 角速度制御
    AttitudeController attitude_controller; // 姿勢制御
    VelocityController velocity_controller; // 速度制御
    PositionController position_controller; // 位置制御
    
    // 出力層
    MotorMixer motor_mixer;
    SafetyMonitor safety_monitor;
    
public:
    void update() {
        // 1. センサ読み取りと状態推定
        IMUData imu_data = imu_processor.read();
        State state = state_estimator.estimate(imu_data);
        
        // 2. カスケード制御
        Vector3 pos_cmd = position_controller.compute(state);
        Vector3 vel_cmd = velocity_controller.compute(state, pos_cmd);
        Quaternion att_cmd = attitude_controller.compute(state, vel_cmd);
        Vector3 rate_cmd = rate_controller.compute(state, att_cmd);
        
        // 3. モータ出力
        MotorCommands motors = motor_mixer.mix(rate_cmd, state.throttle);
        
        // 4. 安全性チェック
        motors = safety_monitor.check(motors, state);
        
        // 5. モータ指令送信
        sendMotorCommands(motors);
    }
};
```

### 5.2.2 FreeRTOSタスクベース制御システム

ESP-IDFのFreeRTOSを活用したリアルタイム制御アーキテクチャ：

```c
// FreeRTOSタスクベースの制御システム
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define SENSOR_TASK_PRIORITY    3
#define CONTROL_TASK_PRIORITY   2
#define COMM_TASK_PRIORITY      1

static const char* TAG = "FlightControl";

// 飛行状態管理クラス
class FlightState {
private:
    float roll_, pitch_, yaw_;
    std::array<float, 3> accel_;
    std::array<float, 3> gyro_;
    std::array<float, 3> position_;
    std::array<float, 3> velocity_;
    mutable std::mutex mutex_;
    
public:
    explicit FlightState() : roll_(0), pitch_(0), yaw_(0) {
        accel_.fill(0);
        gyro_.fill(0);
        position_.fill(0);
        velocity_.fill(0);
    }
    
    // 姿勢データ
    void setAttitude(float roll, float pitch, float yaw) {
        std::lock_guard<std::mutex> lock(mutex_);
        roll_ = roll; pitch_ = pitch; yaw_ = yaw;
    }
    
    std::tuple<float, float, float> getAttitude() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::make_tuple(roll_, pitch_, yaw_);
    }
    
    // 加速度データ
    void setAcceleration(const std::array<float, 3>& accel) {
        std::lock_guard<std::mutex> lock(mutex_);
        accel_ = accel;
    }
    
    std::array<float, 3> getAcceleration() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return accel_;
    }
    
    // 角速度データ
    void setGyroscope(const std::array<float, 3>& gyro) {
        std::lock_guard<std::mutex> lock(mutex_);
        gyro_ = gyro;
    }
    
    std::array<float, 3> getGyroscope() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return gyro_;
    }
    
    // 位置データ
    void setPosition(const std::array<float, 3>& position) {
        std::lock_guard<std::mutex> lock(mutex_);
        position_ = position;
    }
    
    std::array<float, 3> getPosition() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return position_;
    }
    
    // 速度データ
    void setVelocity(const std::array<float, 3>& velocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_ = velocity;
    }
    
    std::array<float, 3> getVelocity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return velocity_;
    }
};

static flight_state_t g_flight_state;
static QueueHandle_t control_queue;

// センサ読み取りタスク（1kHz）
void sensor_task(void *parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(1);  // 1ms
    
    while (1) {
        // センサデータ読み取り
        read_imu_data();
        
        // 共有データを安全に更新
        if (xSemaphoreTake(g_flight_state.mutex, portMAX_DELAY) == pdTRUE) {
            update_flight_state();
            xSemaphoreGive(g_flight_state.mutex);
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// メイン制御タスク（500Hz）
void control_task(void *parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(2);  // 2ms
    
    while (1) {
        control_command_t cmd;
        
        // ESP-NOWからのコマンド受信（ノンブロッキング）
        if (xQueueReceive(control_queue, &cmd, 0) == pdTRUE) {
            process_control_command(&cmd);
        }
        
        // 制御計算実行
        if (xSemaphoreTake(g_flight_state.mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            execute_flight_control();
            xSemaphoreGive(g_flight_state.mutex);
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// ESP-NOW通信タスク（100Hz）
void communication_task(void *parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(10);  // 10ms
    
    while (1) {
        // テレメトリデータ送信
        send_telemetry_data();
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

// システム初期化
void init_flight_control_system(void) {
    // ミューテックスとキューの作成
    g_flight_state.mutex = xSemaphoreCreateMutex();
    control_queue = xQueueCreate(10, sizeof(control_command_t));
    
    // タスク作成
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 
                SENSOR_TASK_PRIORITY, NULL);
    xTaskCreate(control_task, "control", 4096, NULL, 
                CONTROL_TASK_PRIORITY, NULL);
    xTaskCreate(communication_task, "comm", 4096, NULL, 
                COMM_TASK_PRIORITY, NULL);
    
    ESP_LOGI(TAG, "Flight control system initialized");
}
```

### 5.2.3 従来の制御周期管理

```cpp
// 高度マルチレート制御システム
class AdvancedMultiRateController {
private:
    static constexpr int SENSOR_RATE = 1000;     // Hz
    static constexpr int RATE_CTRL_RATE = 500;   // Hz
    static constexpr int ATT_CTRL_RATE = 250;    // Hz
    static constexpr int POS_CTRL_RATE = 50;     // Hz
    
    struct ControlTiming {
        uint32_t last_rate_ctrl = 0;
        uint32_t last_att_ctrl = 0;
        uint32_t last_pos_ctrl = 0;
        uint32_t sensor_count = 0;
        uint32_t control_cycle_count = 0;
    };
    
    ControlTiming timing_;
    
    // 制御器
    std::unique_ptr<RateController> rate_controller_;
    std::unique_ptr<AttitudeController> attitude_controller_;
    std::unique_ptr<PositionController> position_controller_;
    
    // 状態管理
    std::shared_ptr<FlightState> flight_state_;
    
    // 診断機能
    struct PerformanceMetrics {
        float avg_sensor_dt = 0;
        float avg_control_dt = 0;
        uint32_t missed_deadlines = 0;
        float cpu_usage = 0;
    } metrics_;
    
public:
    explicit AdvancedMultiRateController(std::shared_ptr<FlightState> state)
        : flight_state_(std::move(state)) {
        rate_controller_ = std::make_unique<RateController>();
        attitude_controller_ = std::make_unique<AttitudeController>();
        position_controller_ = std::make_unique<PositionController>();
    }
    
    void update(uint32_t current_time) {
        // 性能測定開始
        uint32_t cycle_start = esp_timer_get_time();
        
        // センサは毎回読む（1kHz）
        readSensors();
        timing_.sensor_count++;
        
        // 角速度制御（500Hz）
        if (shouldExecuteRateControl(current_time)) {
            updateRateControl();
            timing_.last_rate_ctrl = current_time;
        }
        
        // 姿勢制御（250Hz）
        if (shouldExecuteAttitudeControl(current_time)) {
            updateAttitudeControl();
            timing_.last_att_ctrl = current_time;
        }
        
        // 位置制御（50Hz）
        if (shouldExecutePositionControl(current_time)) {
            updatePositionControl();
            timing_.last_pos_ctrl = current_time;
        }
        
        // 性能測定終了
        uint32_t cycle_end = esp_timer_get_time();
        updatePerformanceMetrics(cycle_end - cycle_start);
        timing_.control_cycle_count++;
    }
    
    // 制御周期判定
    bool shouldExecuteRateControl(uint32_t current_time) const noexcept {
        return (current_time - timing_.last_rate_ctrl) >= (1000000 / RATE_CTRL_RATE);
    }
    
    bool shouldExecuteAttitudeControl(uint32_t current_time) const noexcept {
        return (current_time - timing_.last_att_ctrl) >= (1000000 / ATT_CTRL_RATE);
    }
    
    bool shouldExecutePositionControl(uint32_t current_time) const noexcept {
        return (current_time - timing_.last_pos_ctrl) >= (1000000 / POS_CTRL_RATE);
    }
    
    // 診断機能
    const PerformanceMetrics& getMetrics() const noexcept { return metrics_; }
    
    void resetMetrics() {
        metrics_ = PerformanceMetrics{};
        timing_.sensor_count = 0;
        timing_.control_cycle_count = 0;
    }
    
private:
    void readSensors() {
        // センサデータ読み取り実装
        // IMU, 磁力計, 気圧計等
    }
    
    void updateRateControl() {
        if (rate_controller_) {
            rate_controller_->update(flight_state_);
        }
    }
    
    void updateAttitudeControl() {
        if (attitude_controller_) {
            attitude_controller_->update(flight_state_);
        }
    }
    
    void updatePositionControl() {
        if (position_controller_) {
            position_controller_->update(flight_state_);
        }
    }
    
    void updatePerformanceMetrics(uint32_t cycle_time) {
        // 移動平均でCPU使用率を計算
        const float alpha = 0.1f;
        metrics_.avg_control_dt = (1.0f - alpha) * metrics_.avg_control_dt + 
                                  alpha * cycle_time;
        
        // デッドライン監視
        if (cycle_time > 2000) {  // 2ms以上の場合
            metrics_.missed_deadlines++;
        }
    }
};
```

## 5.3 センサフュージョンと状態推定

### 5.3.1 相補フィルタ

シンプルで効果的な姿勢推定：

```cpp
class ComplementaryFilter {
private:
    float alpha = 0.98;  // ジャイロの重み
    float roll = 0, pitch = 0, yaw = 0;
    uint32_t last_update = 0;
    
public:
    void update(float ax, float ay, float az, 
                float gx, float gy, float gz) {
        uint32_t now = micros();
        float dt = (now - last_update) * 1e-6;
        last_update = now;
        
        // 加速度から姿勢を計算（重力方向を仮定）
        float roll_acc = atan2(ay, az);
        float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));
        
        // ジャイロによる角度更新
        roll += gx * dt;
        pitch += gy * dt;
        yaw += gz * dt;
        
        // 相補フィルタ
        roll = alpha * roll + (1 - alpha) * roll_acc;
        pitch = alpha * pitch + (1 - alpha) * pitch_acc;
        // ヨーは磁気センサがないため、ジャイロのみ
        
        // 角度の正規化
        normalizeAngles();
    }
    
    void normalizeAngles() {
        while (roll > M_PI) roll -= 2*M_PI;
        while (roll < -M_PI) roll += 2*M_PI;
        while (pitch > M_PI) pitch -= 2*M_PI;
        while (pitch < -M_PI) pitch += 2*M_PI;
        while (yaw > M_PI) yaw -= 2*M_PI;
        while (yaw < -M_PI) yaw += 2*M_PI;
    }
};
```

### 5.3.2 マドウィックフィルタ

より高精度な姿勢推定アルゴリズム：

```cpp
class MadgwickFilter {
private:
    float beta = 0.1;  // フィルタゲイン
    Quaternion q = {1, 0, 0, 0};  // 姿勢四元数
    
public:
    void update(float ax, float ay, float az,
                float gx, float gy, float gz, float dt) {
        // 測定値の正規化
        float norm = sqrt(ax*ax + ay*ay + az*az);
        if (norm == 0) return;
        ax /= norm; ay /= norm; az /= norm;
        
        // 補助変数
        float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2;
        float _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
        
        // 四元数の変化率（ジャイロによる）
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
        
        // 事前計算
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // 勾配降下アルゴリズム
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay 
             - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay 
             - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // 正規化
        norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        norm = 1.0f / norm;
        s0 *= norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        
        // フィードバック適用
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
        
        // 四元数の更新
        q0 += qDot1 * dt;
        q1 += qDot2 * dt;
        q2 += qDot3 * dt;
        q3 += qDot4 * dt;
        
        // 正規化
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        norm = 1.0f / norm;
        q.w = q0 * norm;
        q.x = q1 * norm;
        q.y = q2 * norm;
        q.z = q3 * norm;
    }
    
    void getEulerAngles(float& roll, float& pitch, float& yaw) {
        roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
        pitch = asin(2*(q.w*q.y - q.z*q.x));
        yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    }
};
```

### 5.3.3 拡張センサフュージョン

```cpp
class ExtendedKalmanFilter {
private:
    // 状態ベクトル: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    Eigen::VectorXf state(9);
    Eigen::MatrixXf P(9, 9);  // 共分散行列
    
public:
    void predict(float dt) {
        // IMUデータによる予測ステップ
        // 状態遷移行列の適用
    }
    
    void updateWithMagnetometer(float mx, float my, float mz) {
        // 磁力計による方位更新
        float yaw_mag = atan2(-my, mx);
        
        // 観測行列
        Eigen::MatrixXf H(1, 9);
        H.setZero();
        H(0, 8) = 1;  // ヨー角
        
        // カルマンゲイン計算と更新
        updateKalman(H, yaw_mag - state(8));
    }
    
    void updateWithBarometer(float pressure) {
        // 気圧から高度を計算
        float altitude = 44330 * (1 - pow(pressure / 1013.25, 0.1903));
        
        // 観測行列
        Eigen::MatrixXf H(1, 9);
        H.setZero();
        H(0, 2) = 1;  // z位置
        
        // 更新
        updateKalman(H, altitude - state(2));
    }
    
    void updateWithOpticalFlow(float flow_x, float flow_y, float distance) {
        // オプティカルフローから速度を推定
        float vx_flow = flow_x * distance / PMW3901MBConfig::RESOLUTION;
        float vy_flow = flow_y * distance / PMW3901MBConfig::RESOLUTION;
        
        // 観測行列
        Eigen::MatrixXf H(2, 9);
        H.setZero();
        H(0, 3) = 1;  // vx
        H(1, 4) = 1;  // vy
        
        Eigen::Vector2f innovation;
        innovation << vx_flow - state(3), vy_flow - state(4);
        
        updateKalman(H, innovation);
    }
    
    void updateWithDistanceSensor(float distance1, float distance2) {
        // 2つの距離センサから高度と傾きを推定
        float avg_distance = (distance1 + distance2) / 2.0 / 1000.0;  // m
        float tilt = atan2(distance1 - distance2, VL53L3CXConfig::SENSOR_SPACING);
        
        // 高度更新
        if (avg_distance < VL53L3CXConfig::MAX_RANGE / 1000.0) {
            Eigen::MatrixXf H(1, 9);
            H.setZero();
            H(0, 2) = 1;
            updateKalman(H, avg_distance - state(2));
        }
    }
};
```

### 5.3.4 センサキャリブレーション

```cpp
class IMUCalibration {
private:
    struct CalibrationData {
        float accel_offset[3] = {0, 0, 0};
        float accel_scale[3] = {1, 1, 1};
        float gyro_offset[3] = {0, 0, 0};
    } calib;
    
public:
    void calibrateGyro(int samples = 1000) {
        float sum[3] = {0, 0, 0};
        
        ESP_LOGI(TAG, "Calibrating gyro... Keep drone still!");
        
        for (int i = 0; i < samples; i++) {
            float gx, gy, gz;
            readGyroRaw(&gx, &gy, &gz);
            sum[0] += gx;
            sum[1] += gy;
            sum[2] += gz;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        
        calib.gyro_offset[0] = sum[0] / samples;
        calib.gyro_offset[1] = sum[1] / samples;
        calib.gyro_offset[2] = sum[2] / samples;
        
        ESP_LOGI(TAG, "Gyro calibration complete!");
    }
    
    void calibrateAccel() {
        ESP_LOGI(TAG, "Accel calibration - Place drone level");
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        float ax_level, ay_level, az_level;
        readAccelAverage(&ax_level, &ay_level, &az_level, 100);
        
        // レベル状態でZ軸は1gを示すべき
        calib.accel_offset[0] = ax_level;
        calib.accel_offset[1] = ay_level;
        calib.accel_offset[2] = az_level - 1.0;  // 1g分を引く
        
        ESP_LOGI(TAG, "Accel calibration complete!");
    }
    
    void applyCalibration(float& ax, float& ay, float& az,
                         float& gx, float& gy, float& gz) {
        // 加速度計の補正
        ax = (ax - calib.accel_offset[0]) * calib.accel_scale[0];
        ay = (ay - calib.accel_offset[1]) * calib.accel_scale[1];
        az = (az - calib.accel_offset[2]) * calib.accel_scale[2];
        
        // ジャイロの補正
        gx -= calib.gyro_offset[0];
        gy -= calib.gyro_offset[1];
        gz -= calib.gyro_offset[2];
    }
};
```

## 5.4 姿勢制御の実装

### 5.4.1 角速度制御（最内側ループ）

```cpp
class RateController {
private:
    PIDController roll_rate_pid;
    PIDController pitch_rate_pid;
    PIDController yaw_rate_pid;
    
public:
    RateController() :
        roll_rate_pid(100, 20, 10, 100, 200),    // Kp, Ki, Kd, i_limit, o_limit
        pitch_rate_pid(100, 20, 10, 100, 200),
        yaw_rate_pid(80, 10, 0, 50, 100) {}
    
    Vector3 compute(const State& state, const Vector3& rate_setpoint) {
        float dt = 0.002;  // 500Hz
        
        // 各軸の角速度制御
        float roll_output = roll_rate_pid.update(
            rate_setpoint.x, state.gyro.x, dt);
        float pitch_output = pitch_rate_pid.update(
            rate_setpoint.y, state.gyro.y, dt);
        float yaw_output = yaw_rate_pid.update(
            rate_setpoint.z, state.gyro.z, dt);
        
        return Vector3(roll_output, pitch_output, yaw_output);
    }
    
    void reset() {
        roll_rate_pid.reset();
        pitch_rate_pid.reset();
        yaw_rate_pid.reset();
    }
};
```

### 5.4.2 姿勢制御（中間ループ）

```cpp
class AttitudeController {
private:
    PIDController roll_angle_pid;
    PIDController pitch_angle_pid;
    PIDController yaw_angle_pid;
    
    // 姿勢誤差の計算方法
    enum ErrorMethod {
        EULER_ANGLE,
        QUATERNION
    } error_method = QUATERNION;
    
public:
    AttitudeController() :
        roll_angle_pid(6, 0, 0.15, 50, 250),
        pitch_angle_pid(6, 0, 0.15, 50, 250),
        yaw_angle_pid(4, 0, 0, 50, 100) {}
    
    Vector3 compute(const State& state, const Quaternion& attitude_setpoint) {
        float dt = 0.004;  // 250Hz
        
        if (error_method == QUATERNION) {
            return computeQuaternionError(state, attitude_setpoint, dt);
        } else {
            return computeEulerError(state, attitude_setpoint, dt);
        }
    }
    
private:
    Vector3 computeQuaternionError(const State& state, 
                                  const Quaternion& q_sp, float dt) {
        // 四元数誤差
        Quaternion q_error = q_sp * state.attitude.conjugate();
        
        // 誤差四元数を軸角表現に変換
        float angle_error = 2 * acos(constrain(q_error.w, -1, 1));
        Vector3 axis_error;
        
        if (fabs(angle_error) > 0.001) {
            float s = sin(angle_error / 2);
            axis_error.x = q_error.x / s;
            axis_error.y = q_error.y / s;
            axis_error.z = q_error.z / s;
            
            // 角度誤差を適用
            axis_error *= angle_error;
        }
        
        // PID制御
        float roll_rate = roll_angle_pid.update(0, -axis_error.x, dt);
        float pitch_rate = pitch_angle_pid.update(0, -axis_error.y, dt);
        float yaw_rate = yaw_angle_pid.update(0, -axis_error.z, dt);
        
        return Vector3(roll_rate, pitch_rate, yaw_rate);
    }
    
    Vector3 computeEulerError(const State& state, 
                             const Quaternion& q_sp, float dt) {
        // オイラー角に変換
        float roll_sp, pitch_sp, yaw_sp;
        quaternionToEuler(q_sp, roll_sp, pitch_sp, yaw_sp);
        
        float roll_current, pitch_current, yaw_current;
        quaternionToEuler(state.attitude, roll_current, 
                         pitch_current, yaw_current);
        
        // 角度誤差（-πからπに正規化）
        float roll_error = normalizeAngle(roll_sp - roll_current);
        float pitch_error = normalizeAngle(pitch_sp - pitch_current);
        float yaw_error = normalizeAngle(yaw_sp - yaw_current);
        
        // PID制御
        float roll_rate = roll_angle_pid.update(roll_error, 0, dt);
        float pitch_rate = pitch_angle_pid.update(pitch_error, 0, dt);
        float yaw_rate = yaw_angle_pid.update(yaw_error, 0, dt);
        
        return Vector3(roll_rate, pitch_rate, yaw_rate);
    }
};
```

### 5.4.3 アクロバティックモード

```cpp
class AcroController {
private:
    float max_rate = 720;  // deg/s
    float expo = 0.7;      // エクスポネンシャル
    
public:
    Vector3 compute(const RCInput& rc_input) {
        // スティック入力を角速度指令に変換
        float roll_rate = applyExpo(rc_input.roll) * max_rate;
        float pitch_rate = applyExpo(rc_input.pitch) * max_rate;
        float yaw_rate = applyExpo(rc_input.yaw) * max_rate * 0.5;
        
        return Vector3(roll_rate, pitch_rate, yaw_rate);
    }
    
private:
    float applyExpo(float input) {
        // エクスポネンシャルカーブ
        return input * (fabs(input) * expo + (1 - expo));
    }
};
```

## 5.5 位置・速度制御

### 5.5.1 速度制御

```cpp
class VelocityController {
private:
    PIDController vx_pid;
    PIDController vy_pid;
    PIDController vz_pid;
    
    float max_tilt_angle = 30 * DEG_TO_RAD;  // 最大傾斜角
    
public:
    VelocityController() :
        vx_pid(2, 0.5, 0, 10, max_tilt_angle),
        vy_pid(2, 0.5, 0, 10, max_tilt_angle),
        vz_pid(4, 1, 0.1, 20, 0.3) {}  // 推力の±30%
    
    AttitudeSetpoint compute(const State& state, 
                           const Vector3& velocity_setpoint) {
        float dt = 0.01;  // 100Hz
        
        // 水平速度制御 → 傾斜角
        float pitch_cmd = vx_pid.update(
            velocity_setpoint.x, state.velocity.x, dt);
        float roll_cmd = -vy_pid.update(
            velocity_setpoint.y, state.velocity.y, dt);
        
        // 垂直速度制御 → 推力調整
        float thrust_adjustment = vz_pid.update(
            velocity_setpoint.z, state.velocity.z, dt);
        
        // 姿勢目標の生成
        AttitudeSetpoint att_sp;
        att_sp.roll = roll_cmd;
        att_sp.pitch = pitch_cmd;
        att_sp.yaw = state.yaw;  // 現在のヨーを維持
        att_sp.thrust = state.hover_thrust + thrust_adjustment;
        
        return att_sp;
    }
};
```

### 5.5.2 位置制御

```cpp
class PositionController {
private:
    PIDController x_pid;
    PIDController y_pid;
    PIDController z_pid;
    
    float max_velocity = 5.0;  // m/s
    float max_velocity_z = 2.0;  // m/s
    
public:
    PositionController() :
        x_pid(1, 0.02, 0.5, 5, max_velocity),
        y_pid(1, 0.02, 0.5, 5, max_velocity),
        z_pid(2, 0.05, 1, 5, max_velocity_z) {}
    
    Vector3 compute(const State& state, const Vector3& position_setpoint) {
        float dt = 0.02;  // 50Hz
        
        // 位置誤差から速度指令を生成
        float vx_cmd = x_pid.update(
            position_setpoint.x, state.position.x, dt);
        float vy_cmd = y_pid.update(
            position_setpoint.y, state.position.y, dt);
        float vz_cmd = z_pid.update(
            position_setpoint.z, state.position.z, dt);
        
        // 速度制限
        Vector3 velocity_cmd(vx_cmd, vy_cmd, vz_cmd);
        limitVelocity(velocity_cmd);
        
        return velocity_cmd;
    }
    
private:
    void limitVelocity(Vector3& vel) {
        // 水平速度の制限
        float horizontal_speed = sqrt(vel.x*vel.x + vel.y*vel.y);
        if (horizontal_speed > max_velocity) {
            float scale = max_velocity / horizontal_speed;
            vel.x *= scale;
            vel.y *= scale;
        }
        
        // 垂直速度の制限
        vel.z = constrain(vel.z, -max_velocity_z, max_velocity_z);
    }
};
```

### 5.5.3 経路追従制御

```cpp
class PathFollower {
private:
    struct Waypoint {
        Vector3 position;
        float velocity;
        float yaw;
    };
    
    vector<Waypoint> waypoints;
    int current_waypoint = 0;
    float lookahead_distance = 2.0;  // m
    
public:
    Vector3 computeVelocityCommand(const State& state) {
        if (waypoints.empty()) return Vector3(0, 0, 0);
        
        // 現在のウェイポイントへの距離
        Vector3 to_waypoint = waypoints[current_waypoint].position 
                            - state.position;
        float distance = to_waypoint.magnitude();
        
        // ウェイポイント到達判定
        if (distance < 0.5) {  // 0.5m以内
            current_waypoint++;
            if (current_waypoint >= waypoints.size()) {
                // 経路完了
                return Vector3(0, 0, 0);
            }
        }
        
        // Pure Pursuit アルゴリズム
        Vector3 lookahead_point = computeLookaheadPoint(state);
        Vector3 to_lookahead = lookahead_point - state.position;
        
        // 速度指令の生成
        float desired_speed = waypoints[current_waypoint].velocity;
        Vector3 velocity_cmd = to_lookahead.normalized() * desired_speed;
        
        return velocity_cmd;
    }
    
private:
    Vector3 computeLookaheadPoint(const State& state) {
        // 前方予見点の計算
        Vector3 closest_point = findClosestPointOnPath(state.position);
        Vector3 lookahead = closest_point;
        
        float accumulated_distance = 0;
        int wp_idx = current_waypoint;
        
        while (accumulated_distance < lookahead_distance && 
               wp_idx < waypoints.size() - 1) {
            Vector3 segment = waypoints[wp_idx + 1].position 
                            - waypoints[wp_idx].position;
            float segment_length = segment.magnitude();
            
            if (accumulated_distance + segment_length > lookahead_distance) {
                // このセグメント上に前方予見点がある
                float t = (lookahead_distance - accumulated_distance) 
                        / segment_length;
                lookahead = waypoints[wp_idx].position + segment * t;
                break;
            }
            
            accumulated_distance += segment_length;
            wp_idx++;
        }
        
        return lookahead;
    }
};
```

## 5.6 モータミキシング

### 5.6.1 基本的なミキシング

```cpp
class MotorMixer {
private:
    // ミキシング行列（X配置）
    static constexpr float mixing_matrix[4][4] = {
        // Throttle, Roll, Pitch, Yaw
        { 1, -1,  1, -1 },  // Front Left (CW)
        { 1,  1,  1,  1 },  // Front Right (CCW)
        { 1,  1, -1, -1 },  // Rear Right (CW)
        { 1, -1, -1,  1 }   // Rear Left (CCW)
    };
    
public:
    MotorCommands mix(float throttle, const Vector3& moments) {
        MotorCommands cmd;
        
        // 正規化された入力（-1 to 1）
        float inputs[4] = {
            throttle,      // 0 to 1
            moments.x,     // -1 to 1 (roll)
            moments.y,     // -1 to 1 (pitch)
            moments.z      // -1 to 1 (yaw)
        };
        
        // ミキシング
        for (int i = 0; i < 4; i++) {
            cmd.motors[i] = 0;
            for (int j = 0; j < 4; j++) {
                cmd.motors[i] += mixing_matrix[i][j] * inputs[j];
            }
            
            // スロットル範囲にマッピング
            cmd.motors[i] = mapThrottle(cmd.motors[i]);
        }
        
        return cmd;
    }
    
private:
    float mapThrottle(float normalized) {
        // 正規化値（0-1）をPWM値に変換
        normalized = constrain(normalized, 0, 1);
        return MIN_THROTTLE + normalized * (MAX_THROTTLE - MIN_THROTTLE);
    }
};
```

### 5.6.2 高度なミキシング

```cpp
class AdvancedMotorMixer {
private:
    static constexpr float NOMINAL_VOLTAGE = 4.2;  // 1S高電圧バッテリー定格電圧
    
    struct MotorDynamics {
        float thrust_coefficient;
        float torque_coefficient;
        float response_time;
    };
    
    MotorDynamics motor_dynamics[4];
    float battery_voltage = 4.2;  // 1S高電圧バッテリー満充電時
    
public:
    MotorCommands mix(const ControlOutputs& control) {
        MotorCommands cmd;
        
        // 推力とモーメントの要求値
        float total_thrust = control.thrust;
        Vector3 moments = control.moments;
        
        // バッテリー電圧補償
        float voltage_scale = NOMINAL_VOLTAGE / battery_voltage;
        total_thrust *= voltage_scale;
        
        // 非線形性の補償
        compensateNonlinearity(total_thrust, moments);
        
        // 各モータの推力計算
        float motor_thrusts[4];
        solveMotorThrusts(total_thrust, moments, motor_thrusts);
        
        // 推力からPWMへの変換
        for (int i = 0; i < 4; i++) {
            cmd.motors[i] = thrustToPWM(motor_thrusts[i], i);
        }
        
        // 飽和処理
        handleSaturation(cmd);
        
        return cmd;
    }
    
private:
    void compensateNonlinearity(float& thrust, Vector3& moments) {
        // プロペラの非線形特性を補償
        // 推力は回転数の2乗に比例
        thrust = sqrt(thrust);
        
        // モーメントも同様に補償
        moments.x = sign(moments.x) * sqrt(fabs(moments.x));
        moments.y = sign(moments.y) * sqrt(fabs(moments.y));
        moments.z = sign(moments.z) * sqrt(fabs(moments.z));
    }
    
    void solveMotorThrusts(float total_thrust, const Vector3& moments,
                          float motor_thrusts[4]) {
        // 擬似逆行列を使用した最適化
        // minimize: ||A * motor_thrusts - [thrust; moments]||^2
        
        // 簡略化のため、直接計算
        float arm_length = 0.1;  // m
        float k_thrust = 1.0;
        float k_torque = 0.01;
        
        motor_thrusts[0] = total_thrust/4 - moments.x/(4*arm_length) 
                         + moments.y/(4*arm_length) + moments.z/(4*k_torque);
        motor_thrusts[1] = total_thrust/4 + moments.x/(4*arm_length) 
                         + moments.y/(4*arm_length) - moments.z/(4*k_torque);
        motor_thrusts[2] = total_thrust/4 + moments.x/(4*arm_length) 
                         - moments.y/(4*arm_length) + moments.z/(4*k_torque);
        motor_thrusts[3] = total_thrust/4 - moments.x/(4*arm_length) 
                         - moments.y/(4*arm_length) - moments.z/(4*k_torque);
    }
    
    void handleSaturation(MotorCommands& cmd) {
        // 飽和を防ぐためのスケーリング
        float max_cmd = 0;
        float min_cmd = MAX_THROTTLE;
        
        for (int i = 0; i < 4; i++) {
            max_cmd = max(max_cmd, cmd.motors[i]);
            min_cmd = min(min_cmd, cmd.motors[i]);
        }
        
        if (max_cmd > MAX_THROTTLE) {
            // 上限飽和：全体をスケールダウン
            float scale = MAX_THROTTLE / max_cmd;
            for (int i = 0; i < 4; i++) {
                cmd.motors[i] *= scale;
            }
        } else if (min_cmd < MIN_THROTTLE) {
            // 下限飽和：オフセットを追加
            float offset = MIN_THROTTLE - min_cmd;
            for (int i = 0; i < 4; i++) {
                cmd.motors[i] += offset;
            }
        }
    }
};
```

## 5.7 安全機能とフェイルセーフ

### 5.7.1 安全監視システム

```cpp
class SafetyMonitor {
private:
    enum SafetyState {
        SAFE,
        WARNING,
        CRITICAL,
        EMERGENCY
    } safety_state = SAFE;
    
    struct SafetyLimits {
        float max_angle = 45 * DEG_TO_RAD;
        float max_altitude = 100;  // m
        float min_battery = 3.7;   // V (1S high-voltage battery)
        float max_velocity = 10;   // m/s
        float geofence_radius = 50; // m
    } limits;
    
public:
    bool checkSafety(const State& state) {
        safety_state = SAFE;
        
        // 姿勢チェック
        if (fabs(state.roll) > limits.max_angle || 
            fabs(state.pitch) > limits.max_angle) {
            safety_state = CRITICAL;
            return false;
        }
        
        // バッテリーチェック
        if (state.battery_voltage < limits.min_battery) {
            if (state.battery_voltage < 3.5) {  // 1S高電圧バッテリーの緊急閾値
                safety_state = EMERGENCY;
                return false;
            }
            safety_state = WARNING;
        }
        
        // 高度チェック
        if (state.position.z > limits.max_altitude) {
            safety_state = WARNING;
        }
        
        // 速度チェック
        float velocity = state.velocity.magnitude();
        if (velocity > limits.max_velocity) {
            safety_state = WARNING;
        }
        
        // ジオフェンス
        float distance = sqrt(state.position.x * state.position.x + 
                            state.position.y * state.position.y);
        if (distance > limits.geofence_radius) {
            safety_state = CRITICAL;
            return false;
        }
        
        return safety_state != EMERGENCY;
    }
    
    void executeFailsafe(FlightMode& mode) {
        switch (safety_state) {
            case WARNING:
                // 警告表示
                setLED(YELLOW);
                break;
                
            case CRITICAL:
                // 自動レベリング
                mode = STABILIZE;
                setLED(ORANGE);
                break;
                
            case EMERGENCY:
                // 緊急着陸
                mode = EMERGENCY_LAND;
                setLED(RED);
                break;
                
            default:
                setLED(GREEN);
                break;
        }
    }
};
```

### 5.7.2 緊急着陸

```cpp
class EmergencyLanding {
private:
    float descent_rate = 1.0;  // m/s
    float emergency_descent_rate = 2.0;  // m/s
    bool landing_initiated = false;
    
public:
    ControlOutputs execute(const State& state) {
        if (!landing_initiated) {
            landing_initiated = true;
            ESP_LOGW(TAG, "EMERGENCY LANDING INITIATED!");
        }
        
        ControlOutputs control;
        
        // レベル飛行を維持
        control.moments.x = -state.roll * 5;    // 強いレベリング
        control.moments.y = -state.pitch * 5;
        control.moments.z = -state.gyro.z * 2;  // ヨーレート減衰
        
        // 降下率制御
        float target_descent_rate = descent_rate;
        if (state.battery_voltage < 3.4) {
            // バッテリー切れ間近（1S高電圧バッテリー）
            target_descent_rate = emergency_descent_rate;
        }
        
        // 高度に応じた降下率調整
        if (state.position.z < 2.0) {
            // 地面近く：ゆっくり降下
            target_descent_rate *= 0.3;
        }
        
        // 推力計算
        float thrust_adjustment = (target_descent_rate + state.velocity.z) * 0.1;
        control.thrust = HOVER_THRUST - thrust_adjustment;
        
        // 地面検出
        if (state.position.z < 0.1 && fabs(state.velocity.z) < 0.1) {
            // 着陸完了
            control.thrust = 0;
            disarmMotors();
        }
        
        return control;
    }
};
```

### 5.7.3 包括的安全管理システム

要件定義書に基づいた包括的な安全管理システムの実装：

```c
// ESP-IDF安全管理システム
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char* SAFETY_TAG = "SAFETY";

// 安全イベント定義
typedef enum {
    SAFETY_EVENT_BATTERY_LOW = 0x01,
    SAFETY_EVENT_BATTERY_CRITICAL = 0x02,
    SAFETY_EVENT_COMM_LOSS = 0x04,
    SAFETY_EVENT_SENSOR_FAILURE = 0x08,
    SAFETY_EVENT_CONTROL_FAILURE = 0x10,
    SAFETY_EVENT_GEOFENCE_BREACH = 0x20,
    SAFETY_EVENT_ALTITUDE_LIMIT = 0x40,
    SAFETY_EVENT_VELOCITY_LIMIT = 0x80
} safety_event_t;

// 安全状態構造体
typedef struct {
    uint32_t active_events;         // アクティブな安全イベント
    uint64_t last_comm_time;        // 最後の通信時刻 (μs)
    uint64_t last_control_time;     // 最後の制御更新時刻 (μs)
    float battery_voltage;          // バッテリー電圧 (V)
    float flight_time;              // 飛行時間 (s)
    bool motors_armed;              // モータアーム状態
    bool safety_pilot_override;     // セーフティパイロット介入
} safety_state_t;

static safety_state_t g_safety_state = {0};
static QueueHandle_t safety_event_queue;

// 安全イベント処理タスク
void safety_monitor_task(void *parameters) {
    safety_event_t event;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(10);  // 100Hz
    
    while (1) {
        // 定期安全チェック
        perform_periodic_safety_checks();
        
        // イベントキューからの緊急イベント処理
        while (xQueueReceive(safety_event_queue, &event, 0) == pdTRUE) {
            handle_safety_event(event);
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

void perform_periodic_safety_checks(void) {
    uint64_t current_time = esp_timer_get_time();
    
    // 通信タイムアウトチェック
    if (g_safety_state.motors_armed && 
        (current_time - g_safety_state.last_comm_time) > 500000) {  // 500ms
        ESP_LOGE(SAFETY_TAG, "Communication timeout detected!");
        trigger_safety_event(SAFETY_EVENT_COMM_LOSS);
    }
    
    // 制御ループタイムアウトチェック
    if (g_safety_state.motors_armed && 
        (current_time - g_safety_state.last_control_time) > 5000) {  // 5ms
        ESP_LOGE(SAFETY_TAG, "Control loop timeout detected!");
        trigger_safety_event(SAFETY_EVENT_CONTROL_FAILURE);
    }
    
    // バッテリー電圧チェック
    check_battery_safety();
    
    // 飛行時間制限チェック
    if (g_safety_state.flight_time > 300.0) {  // 5分制限
        ESP_LOGW(SAFETY_TAG, "Flight time limit approaching");
        trigger_safety_event(SAFETY_EVENT_BATTERY_LOW);
    }
    
    // プロペラ脱落検知（StampFly特有の安全チェック）
    check_propeller_detachment();
}

void check_battery_safety(void) {
    float voltage = g_safety_state.battery_voltage;
    
    // 1S高電圧バッテリー（4.35V満充電）の安全閾値
    if (voltage < 3.4) {  // 緊急閾値
        ESP_LOGE(SAFETY_TAG, "CRITICAL: Battery voltage %.2fV - Emergency landing!", voltage);
        trigger_safety_event(SAFETY_EVENT_BATTERY_CRITICAL);
    } else if (voltage < 3.6) {  // 警告閾値
        ESP_LOGW(SAFETY_TAG, "WARNING: Battery voltage %.2fV - Return to land", voltage);
        trigger_safety_event(SAFETY_EVENT_BATTERY_LOW);
    }
    
    // セル電圧不均衡チェック（1Sなので単一セル）
    // 内部抵抗増加の検出
    static float last_voltage = 4.2;
    static uint64_t last_check_time = 0;
    uint64_t now = esp_timer_get_time();
    
    if (now - last_check_time > 1000000) {  // 1秒間隔
        float voltage_drop_rate = (last_voltage - voltage) / ((now - last_check_time) / 1000000.0);
        
        if (voltage_drop_rate > 0.1) {  // 0.1V/s以上の急激な電圧降下
            ESP_LOGW(SAFETY_TAG, "Rapid battery discharge detected: %.3fV/s", voltage_drop_rate);
        }
        
        last_voltage = voltage;
        last_check_time = now;
    }
}

void trigger_safety_event(safety_event_t event) {
    g_safety_state.active_events |= event;
    
    // 高優先度キューに送信
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(safety_event_queue, &event, &higher_priority_task_woken);
    
    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

void handle_safety_event(safety_event_t event) {
    ESP_LOGE(SAFETY_TAG, "Safety event triggered: 0x%02X", event);
    
    switch (event) {
        case SAFETY_EVENT_BATTERY_CRITICAL:
            execute_emergency_landing();
            break;
            
        case SAFETY_EVENT_COMM_LOSS:
            execute_communication_loss_procedure();
            break;
            
        case SAFETY_EVENT_SENSOR_FAILURE:
            execute_sensor_failure_procedure();
            break;
            
        case SAFETY_EVENT_CONTROL_FAILURE:
            execute_control_failure_procedure();
            break;
            
        case SAFETY_EVENT_GEOFENCE_BREACH:
            execute_geofence_return();
            break;
            
        default:
            execute_safe_mode();
            break;
    }
}
```

### 5.7.4 ハードウェア故障検出

```c
// センサ故障検出システム
typedef struct {
    bool imu_healthy;
    bool mag_healthy;
    bool baro_healthy;
    bool flow_healthy;
    bool distance_healthy;
    uint32_t imu_error_count;
    uint32_t mag_error_count;
    uint32_t baro_error_count;
    uint64_t last_imu_update;
    uint64_t last_mag_update;
    uint64_t last_baro_update;
} sensor_health_t;

static sensor_health_t g_sensor_health = {0};

void monitor_sensor_health(void) {
    uint64_t current_time = esp_timer_get_time();
    
    // IMU健全性チェック
    if (current_time - g_sensor_health.last_imu_update > 2000) {  // 2ms
        g_sensor_health.imu_error_count++;
        if (g_sensor_health.imu_error_count > 10) {
            g_sensor_health.imu_healthy = false;
            ESP_LOGE(SAFETY_TAG, "IMU sensor failure detected!");
            trigger_safety_event(SAFETY_EVENT_SENSOR_FAILURE);
        }
    } else {
        g_sensor_health.imu_error_count = 0;
        g_sensor_health.imu_healthy = true;
    }
    
    // 磁力計健全性チェック
    if (current_time - g_sensor_health.last_mag_update > 100000) {  // 100ms
        g_sensor_health.mag_error_count++;
        if (g_sensor_health.mag_error_count > 5) {
            g_sensor_health.mag_healthy = false;
            ESP_LOGW(SAFETY_TAG, "Magnetometer sensor degraded");
        }
    } else {
        g_sensor_health.mag_error_count = 0;
        g_sensor_health.mag_healthy = true;
    }
    
    // 気圧センサ健全性チェック  
    if (current_time - g_sensor_health.last_baro_update > 50000) {  // 50ms
        g_sensor_health.baro_error_count++;
        if (g_sensor_health.baro_error_count > 3) {
            g_sensor_health.baro_healthy = false;
            ESP_LOGW(SAFETY_TAG, "Barometer sensor degraded");
        }
    } else {
        g_sensor_health.baro_error_count = 0;
        g_sensor_health.baro_healthy = true;
    }
}

// センサデータ検証
bool validate_sensor_data(const imu_data_t* imu, const baro_data_t* baro) {
    // IMUデータの妥当性チェック
    float accel_magnitude = sqrt(imu->accel_x * imu->accel_x + 
                                imu->accel_y * imu->accel_y + 
                                imu->accel_z * imu->accel_z);
    
    // 重力加速度から大きく外れている場合は異常
    if (accel_magnitude < 5.0 || accel_magnitude > 15.0) {
        ESP_LOGW(SAFETY_TAG, "Abnormal accelerometer reading: %.2f m/s²", accel_magnitude);
        return false;
    }
    
    // ジャイロの妥当性チェック（静止時の基準）
    if (fabs(imu->gyro_x) > 10.0 || fabs(imu->gyro_y) > 10.0 || fabs(imu->gyro_z) > 10.0) {
        // 高角速度は正常な場合もあるため、継続監視
        static uint32_t high_gyro_count = 0;
        high_gyro_count++;
        
        if (high_gyro_count > 1000) {  // 1秒間継続
            ESP_LOGW(SAFETY_TAG, "Sustained high angular rates detected");
            high_gyro_count = 0;
        }
    }
    
    // 気圧データの妥当性チェック
    if (baro->pressure < 800.0 || baro->pressure > 1200.0) {  // hPa
        ESP_LOGW(SAFETY_TAG, "Abnormal pressure reading: %.1f hPa", baro->pressure);
        return false;
    }
    
    return true;
}
```

### 5.7.5 通信途絶対応

```c
// 通信途絶時の自動対応システム
typedef enum {
    COMM_LOSS_ACTION_HOVER,
    COMM_LOSS_ACTION_RTL,
    COMM_LOSS_ACTION_LAND,
    COMM_LOSS_ACTION_TERMINATE
} comm_loss_action_t;

typedef struct {
    comm_loss_action_t action;
    uint32_t timeout_ms;
    float rtl_altitude;
    bool enable_geofence;
} comm_loss_config_t;

static comm_loss_config_t g_comm_loss_config = {
    .action = COMM_LOSS_ACTION_RTL,
    .timeout_ms = 500,  // 500ms
    .rtl_altitude = 20.0,  // 20m
    .enable_geofence = true
};

void execute_communication_loss_procedure(void) {
    ESP_LOGW(SAFETY_TAG, "Executing communication loss procedure");
    
    static uint64_t comm_loss_start_time = 0;
    uint64_t current_time = esp_timer_get_time();
    
    if (comm_loss_start_time == 0) {
        comm_loss_start_time = current_time;
    }
    
    uint32_t loss_duration = (current_time - comm_loss_start_time) / 1000;  // ms
    
    switch (g_comm_loss_config.action) {
        case COMM_LOSS_ACTION_HOVER:
            // 最後の位置でホバリング
            maintain_hover_position();
            
            // 一定時間後に着陸
            if (loss_duration > 10000) {  // 10秒後
                execute_safe_landing();
            }
            break;
            
        case COMM_LOSS_ACTION_RTL:
            // Return to Launch
            execute_return_to_launch();
            break;
            
        case COMM_LOSS_ACTION_LAND:
            // その場で着陸
            execute_safe_landing();
            break;
            
        case COMM_LOSS_ACTION_TERMINATE:
            // 緊急モータ停止（最後の手段）
            if (loss_duration > 30000) {  // 30秒後
                emergency_motor_cutoff();
            }
            break;
    }
}

void maintain_hover_position(void) {
    // 最後に受信した位置指令を維持
    static bool position_locked = false;
    static Vector3 hold_position;
    
    if (!position_locked) {
        hold_position = get_current_position();
        position_locked = true;
        ESP_LOGI(SAFETY_TAG, "Position locked at %.2f, %.2f, %.2f", 
                hold_position.x, hold_position.y, hold_position.z);
    }
    
    // 位置制御を継続
    set_position_setpoint(hold_position);
}

// 通信復旧検出
void check_communication_recovery(void) {
    static bool comm_lost = false;
    uint64_t current_time = esp_timer_get_time();
    
    bool comm_active = (current_time - g_safety_state.last_comm_time) < 100000;  // 100ms
    
    if (comm_lost && comm_active) {
        ESP_LOGI(SAFETY_TAG, "Communication recovered!");
        comm_lost = false;
        
        // 安全イベントをクリア
        g_safety_state.active_events &= ~SAFETY_EVENT_COMM_LOSS;
        
        // 通常制御に復帰（パイロット判断）
        // ただし、自動的には戻さず、明示的な指令を待つ
    } else if (!comm_lost && !comm_active) {
        ESP_LOGW(SAFETY_TAG, "Communication lost!");
        comm_lost = true;
    }
}

// StampFly特有のプロペラ脱落検知
void check_propeller_detachment(void) {
    static float last_motor_commands[4] = {0};
    static float last_gyro_magnitude = 0;
    
    extern MotorCommands current_motor_commands;
    extern imu_data_t current_imu_data;
    
    // 現在のモータ出力合計
    float total_motor_output = 0;
    for (int i = 0; i < 4; i++) {
        float normalized = (current_motor_commands.motors[i] - 1000) / 1000.0;
        total_motor_output += normalized;
    }
    
    // 角速度の大きさ
    float gyro_magnitude = sqrt(current_imu_data.gyro_x * current_imu_data.gyro_x +
                               current_imu_data.gyro_y * current_imu_data.gyro_y +
                               current_imu_data.gyro_z * current_imu_data.gyro_z);
    
    // プロペラ脱落の兆候検出
    // StampFly個体差によるプロペラ緩み対策
    if (total_motor_output > 0.3 && g_safety_state.motors_armed) {  // 30%以上の出力時
        // 急激な角速度変化（プロペラ脱落による推力不均衡）
        float gyro_change_rate = fabs(gyro_magnitude - last_gyro_magnitude);
        
        if (gyro_change_rate > 5.0) {  // 5 rad/s以上の急変
            static uint32_t anomaly_count = 0;
            anomaly_count++;
            
            if (anomaly_count > 5) {  // 50ms間継続
                ESP_LOGE(SAFETY_TAG, "CRITICAL: Propeller detachment suspected!");
                ESP_LOGE(SAFETY_TAG, "Motor output: %.1f%%, Gyro change: %.2f rad/s", 
                        total_motor_output * 100, gyro_change_rate);
                
                // 即座にモータ停止（プロペラが上方向に飛散する危険回避）
                emergency_motor_cutoff();
                anomaly_count = 0;
            }
        } else {
            // 正常状態では異常カウンタをリセット
            static uint32_t anomaly_count = 0;
            anomaly_count = 0;
        }
    }
    
    // 値を保存
    memcpy(last_motor_commands, current_motor_commands.motors, sizeof(last_motor_commands));
    last_gyro_magnitude = gyro_magnitude;
}

// 緊急モータ遮断（プロペラ脱落時の安全措置）
void emergency_motor_cutoff(void) {
    ESP_LOGE(SAFETY_TAG, "EMERGENCY MOTOR CUTOFF - PROPELLER SAFETY!");
    
    // 全モータを即座に停止
    MotorCommands stop_cmd = {0};
    for (int i = 0; i < 4; i++) {
        stop_cmd.motors[i] = 1000;  // 最小スロットル
    }
    
    // モータ出力を強制的に停止
    sendMotorCommands(stop_cmd);
    
    // アーム状態を解除
    g_safety_state.motors_armed = false;
    
    // 緊急停止フラグ設定
    g_safety_state.active_events |= SAFETY_EVENT_CONTROL_FAILURE;
    
    // プロペラ脱落警告
    ESP_LOGW(SAFETY_TAG, "WARNING: Check propeller attachment before next flight");
}
```

### 5.7.6 高度バッテリー管理システム

M5StampFlyの300mAh 1S高電圧バッテリーに最適化された管理システム：

```c
// 1S高電圧リチウムポリマーバッテリー管理
#include "driver/adc.h"
#include "esp_adc_cal.h"

// バッテリー特性（300mAh 1S HV LiPo）
#define BATTERY_CAPACITY_MAH    300
#define BATTERY_VOLTAGE_MAX     4.35    // V (HV LiPo満充電)
#define BATTERY_VOLTAGE_MIN     3.0     // V (絶対最小値)
#define BATTERY_VOLTAGE_NOMINAL 3.7     // V (公称電圧)
#define BATTERY_CUTOFF_VOLTAGE  3.4     // V (カットオフ電圧)
#define BATTERY_WARNING_VOLTAGE 3.6     // V (警告電圧)

typedef struct {
    float voltage;              // 現在電圧 (V)
    float current;              // 現在電流 (A) - 負値は放電
    float remaining_capacity;   // 残量 (mAh)
    float remaining_percentage; // 残量率 (%)
    float internal_resistance;  // 内部抵抗 (Ω)
    float temperature;          // 温度 (℃)
    uint32_t cycle_count;       // 充放電サイクル数
    uint32_t total_flight_time; // 総飛行時間 (s)
    bool is_charging;           // 充電中フラグ
    bool health_good;           // バッテリー健全性
} battery_state_t;

static battery_state_t g_battery_state = {0};
static esp_adc_cal_characteristics_t adc_chars;

// バッテリー管理タスク
void battery_management_task(void *parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(100);  // 10Hz
    
    while (1) {
        // バッテリー状態更新
        update_battery_state();
        
        // 残量計算
        calculate_remaining_capacity();
        
        // 健全性チェック
        check_battery_health();
        
        // 安全性チェック
        check_battery_safety_limits();
        
        // 飛行時間推定
        estimate_remaining_flight_time();
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

void update_battery_state(void) {
    // ADCから電圧読み取り
    uint32_t voltage_raw = 0;
    for (int i = 0; i < 64; i++) {  // 64サンプル平均
        voltage_raw += adc1_get_raw(ADC1_CHANNEL_0);
    }
    voltage_raw /= 64;
    
    // 電圧変換（分圧回路補正）
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(voltage_raw, &adc_chars);
    g_battery_state.voltage = voltage_mv * 2.0 / 1000.0;  // 1/2分圧の補正
    
    // 電流測定（シャント抵抗使用時）
    // 注：M5StampFlyは電流センサなしのため推定
    estimate_current_consumption();
    
    // 温度推定（周囲温度 + 発熱）
    g_battery_state.temperature = 25.0 + (g_battery_state.current * 2.0);
}

void estimate_current_consumption(void) {
    // モータ出力から電流消費を推定
    extern MotorCommands current_motor_commands;
    
    float total_throttle = 0;
    for (int i = 0; i < 4; i++) {
        float normalized = (current_motor_commands.motors[i] - 1000) / 1000.0;
        total_throttle += normalized;
    }
    
    // 経験的な電流推定式（ホバリング時約2A、最大約8A）
    float base_current = 0.5;  // 基本消費電流
    float motor_current = total_throttle * 1.8;  // モータ電流
    
    g_battery_state.current = base_current + motor_current;
}

void calculate_remaining_capacity(void) {
    // 電圧ベースの残量推定（HV LiPoカーブ）
    float voltage = g_battery_state.voltage;
    float percentage;
    
    if (voltage >= 4.2) {
        percentage = 90 + (voltage - 4.2) * 66.7;  // 90-100%
    } else if (voltage >= 4.0) {
        percentage = 70 + (voltage - 4.0) * 100;   // 70-90%
    } else if (voltage >= 3.8) {
        percentage = 40 + (voltage - 3.8) * 150;   // 40-70%
    } else if (voltage >= 3.6) {
        percentage = 15 + (voltage - 3.6) * 125;   // 15-40%
    } else if (voltage >= 3.4) {
        percentage = 5 + (voltage - 3.4) * 50;     // 5-15%
    } else {
        percentage = 0 + (voltage - 3.0) * 12.5;   // 0-5%
    }
    
    // 電流補正（負荷時の電圧降下考慮）
    float voltage_drop = g_battery_state.current * g_battery_state.internal_resistance;
    float corrected_voltage = voltage + voltage_drop;
    
    // クーロンカウンティング（積分）
    static float coulomb_count = BATTERY_CAPACITY_MAH;
    static uint64_t last_update_time = 0;
    
    uint64_t current_time = esp_timer_get_time();
    if (last_update_time > 0) {
        float dt_hours = (current_time - last_update_time) / 3600000000.0;
        coulomb_count -= g_battery_state.current * dt_hours * 1000;  // mAh
    }
    last_update_time = current_time;
    
    // 電圧ベースとクーロンカウンティングの融合
    float voltage_based_mah = percentage * BATTERY_CAPACITY_MAH / 100.0;
    g_battery_state.remaining_capacity = 0.7 * voltage_based_mah + 0.3 * coulomb_count;
    g_battery_state.remaining_percentage = g_battery_state.remaining_capacity / BATTERY_CAPACITY_MAH * 100.0;
    
    // 範囲制限
    g_battery_state.remaining_percentage = fmax(0, fmin(100, g_battery_state.remaining_percentage));
}

void check_battery_health(void) {
    static uint32_t health_check_count = 0;
    health_check_count++;
    
    bool health_issues = false;
    
    // 内部抵抗チェック（健全なHV LiPoは約50mΩ）
    if (g_battery_state.internal_resistance > 0.2) {  // 200mΩ
        ESP_LOGW("BATTERY", "High internal resistance: %.1f mΩ", 
                g_battery_state.internal_resistance * 1000);
        health_issues = true;
    }
    
    // 温度チェック
    if (g_battery_state.temperature > 60.0) {
        ESP_LOGE("BATTERY", "Battery overheating: %.1f°C", g_battery_state.temperature);
        health_issues = true;
    }
    
    // 電圧異常チェック
    if (g_battery_state.voltage > 4.4) {  // 過充電
        ESP_LOGE("BATTERY", "Battery overvoltage: %.2fV", g_battery_state.voltage);
        health_issues = true;
    }
    
    // 急激な電圧降下チェック
    static float last_voltage = 4.2;
    float voltage_drop_rate = (last_voltage - g_battery_state.voltage) / 0.1;  // V/s
    
    if (voltage_drop_rate > 0.5) {  // 0.5V/s以上の急降下
        ESP_LOGW("BATTERY", "Rapid voltage drop: %.2f V/s", voltage_drop_rate);
        health_issues = true;
    }
    last_voltage = g_battery_state.voltage;
    
    // サイクル数チェック（HV LiPoは約300-500サイクル）
    if (g_battery_state.cycle_count > 400) {
        ESP_LOGW("BATTERY", "High cycle count: %lu", g_battery_state.cycle_count);
        health_issues = true;
    }
    
    g_battery_state.health_good = !health_issues;
    
    // 定期的な健全性レポート
    if (health_check_count % 600 == 0) {  // 1分毎
        ESP_LOGI("BATTERY", "Health: %s, Voltage: %.2fV, Capacity: %.1f%%, Resistance: %.1fmΩ",
                g_battery_state.health_good ? "Good" : "Degraded",
                g_battery_state.voltage,
                g_battery_state.remaining_percentage,
                g_battery_state.internal_resistance * 1000);
    }
}

void check_battery_safety_limits(void) {
    // 緊急レベル
    if (g_battery_state.voltage < BATTERY_CUTOFF_VOLTAGE) {
        ESP_LOGE("BATTERY", "CRITICAL: Battery voltage %.2fV below cutoff!", g_battery_state.voltage);
        trigger_safety_event(SAFETY_EVENT_BATTERY_CRITICAL);
        return;
    }
    
    // 警告レベル
    if (g_battery_state.voltage < BATTERY_WARNING_VOLTAGE) {
        ESP_LOGW("BATTERY", "WARNING: Low battery %.2fV (%.1f%%)", 
                g_battery_state.voltage, g_battery_state.remaining_percentage);
        trigger_safety_event(SAFETY_EVENT_BATTERY_LOW);
    }
    
    // 残量ベースの警告
    if (g_battery_state.remaining_percentage < 20.0) {
        ESP_LOGW("BATTERY", "Low battery capacity: %.1f%%", g_battery_state.remaining_percentage);
        trigger_safety_event(SAFETY_EVENT_BATTERY_LOW);
    }
}

float estimate_remaining_flight_time(void) {
    if (g_battery_state.current <= 0) {
        return 0;  // 電流が流れていない
    }
    
    // 残容量から飛行時間を推定
    float remaining_mah = g_battery_state.remaining_capacity;
    float current_ma = g_battery_state.current * 1000;
    
    // 安全マージンを考慮（80%まで使用）
    float usable_mah = remaining_mah * 0.8;
    
    // 推定飛行時間（時間）
    float estimated_hours = usable_mah / current_ma;
    float estimated_minutes = estimated_hours * 60;
    
    // RTL用予備時間を差し引き
    float rtl_reserve_minutes = 1.0;  // 1分の予備
    float available_minutes = estimated_minutes - rtl_reserve_minutes;
    
    ESP_LOGD("BATTERY", "Estimated flight time: %.1f min (%.1f%% remaining)", 
             available_minutes, g_battery_state.remaining_percentage);
    
    return fmax(0, available_minutes);
}

// バッテリー保護機能
void battery_protection_init(void) {
    // ADC設定
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    
    // ADC校正
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 
                            1100, &adc_chars);
    
    // 過放電保護回路の初期化（ハードウェア依存）
    // M5StampFlyの場合、ソフトウェア保護のみ
    
    ESP_LOGI("BATTERY", "Battery protection system initialized");
}

// バッテリー状態の取得
battery_state_t get_battery_state(void) {
    return g_battery_state;
}

// 充電状態の検出
bool is_battery_charging(void) {
    // USB電源接続時の検出ロジック
    // M5StampFlyでは簡易的に電圧上昇で判定
    static float last_voltage = 0;
    static uint32_t voltage_rise_count = 0;
    
    if (g_battery_state.voltage > last_voltage + 0.01) {  // 10mV上昇
        voltage_rise_count++;
        if (voltage_rise_count > 10) {  // 1秒間継続上昇
            g_battery_state.is_charging = true;
        }
    } else {
        voltage_rise_count = 0;
        if (g_battery_state.voltage < 4.0) {  // 充電完了判定
            g_battery_state.is_charging = false;
        }
    }
    
    last_voltage = g_battery_state.voltage;
    return g_battery_state.is_charging;
}
```

## 5.8 ESP-NOW通信とコマンド処理

### 5.8.1 ESP-NOW通信実装

ESP-NOWによる低遅延・高信頼性通信：

```c
#include "esp_now.h"
#include "esp_wifi.h"

// ESP-NOW通信構造体
typedef struct {
    uint8_t msg_type;
    uint32_t timestamp;
    union {
        struct {
            float roll, pitch, yaw, throttle;
            uint8_t arm_switch;
            uint8_t flight_mode;
        } control_data;
        struct {
            float position[3];
            float velocity[3]; 
            float attitude[3];
            float battery_voltage;
            uint8_t flight_status;
        } telemetry_data;
    };
} esp_now_message_t;

// メッセージタイプ定義
#define MSG_CONTROL     0x01
#define MSG_TELEMETRY   0x02
#define MSG_STATUS      0x03

static uint8_t controller_mac[6] = {0x24, 0x6F, 0x28, 0xB5, 0x2C, 0x10};
static bool esp_now_initialized = false;

// ESP-NOW送信コールバック
void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD("ESPNOW", "Send success to " MACSTR, MAC2STR(mac_addr));
    } else {
        ESP_LOGW("ESPNOW", "Send failed to " MACSTR, MAC2STR(mac_addr));
    }
}

// ESP-NOW受信コールバック
void esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len != sizeof(esp_now_message_t)) {
        ESP_LOGW("ESPNOW", "Invalid message length: %d", len);
        return;
    }
    
    esp_now_message_t *msg = (esp_now_message_t *)data;
    
    switch (msg->msg_type) {
        case MSG_CONTROL:
            // 制御コマンドをキューに送信
            if (xQueueSend(control_queue, &msg->control_data, 0) != pdTRUE) {
                ESP_LOGW("ESPNOW", "Control queue full");
            }
            break;
            
        case MSG_STATUS:
            ESP_LOGI("ESPNOW", "Status message received");
            break;
            
        default:
            ESP_LOGW("ESPNOW", "Unknown message type: %d", msg->msg_type);
            break;
    }
}

// ESP-NOW初期化
esp_err_t esp_now_init_drone(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // ESP-NOW初期化
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    // ピア（コントローラ）を追加
    esp_now_peer_info_t peer_info = {};
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, controller_mac, 6);
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    
    esp_now_initialized = true;
    ESP_LOGI("ESPNOW", "ESP-NOW initialized successfully");
    
    return ESP_OK;
}

// テレメトリデータ送信
esp_err_t send_telemetry_esp_now(const flight_state_t *state) {
    if (!esp_now_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_now_message_t msg = {
        .msg_type = MSG_TELEMETRY,
        .timestamp = esp_timer_get_time() / 1000  // ms
    };
    
    // 機体状態をメッセージに詰める
    msg.telemetry_data.position[0] = state->position[0];
    msg.telemetry_data.position[1] = state->position[1]; 
    msg.telemetry_data.position[2] = state->position[2];
    msg.telemetry_data.velocity[0] = state->velocity[0];
    msg.telemetry_data.velocity[1] = state->velocity[1];
    msg.telemetry_data.velocity[2] = state->velocity[2];
    msg.telemetry_data.attitude[0] = state->roll * 180.0f / M_PI;
    msg.telemetry_data.attitude[1] = state->pitch * 180.0f / M_PI;
    msg.telemetry_data.attitude[2] = state->yaw * 180.0f / M_PI;
    
    // ESP-NOW送信
    esp_err_t ret = esp_now_send(controller_mac, (uint8_t *)&msg, sizeof(msg));
    
    if (ret != ESP_OK) {
        ESP_LOGW("ESPNOW", "Send telemetry failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// 制御コマンド受信処理
void process_control_commands(void) {
    esp_now_message_t::control_data cmd;
    
    while (xQueueReceive(control_queue, &cmd, 0) == pdTRUE) {
        // アーム/ディスアーム処理
        if (cmd.arm_switch != g_flight_state.armed) {
            if (cmd.arm_switch && safety_check_passed()) {
                arm_motors();
                ESP_LOGI("FLIGHT", "Motors armed");
            } else {
                disarm_motors();
                ESP_LOGI("FLIGHT", "Motors disarmed");
            }
        }
        
        // フライトモード切替
        if (cmd.flight_mode != g_flight_state.flight_mode) {
            set_flight_mode(cmd.flight_mode);
        }
        
        // 制御入力更新
        update_control_setpoints(cmd.roll, cmd.pitch, cmd.yaw, cmd.throttle);
    }
}
```

### 5.8.2 コマンドプロトコル

```cpp
class CommandProcessor {
private:
    enum CommandType {
        CMD_ARM_DISARM = 0x01,
        CMD_TAKEOFF = 0x02,
        CMD_LAND = 0x03,
        CMD_SET_MODE = 0x04,
        CMD_GOTO = 0x05,
        CMD_SET_HOME = 0x06,
        CMD_RTL = 0x07,
        CMD_EMERGENCY_STOP = 0xFF
    };
    
    struct CommandPacket {
        uint8_t type;
        uint8_t sequence;
        uint16_t checksum;
        union {
            struct { uint8_t arm; } arm_disarm;
            struct { float altitude; } takeoff;
            struct { uint8_t mode; } set_mode;
            struct { float x, y, z, yaw; } goto_pos;
        } data;
    } __attribute__((packed));
    
public:
    void processCommand(const CommandPacket& cmd, FlightController& fc) {
        // チェックサム検証
        if (!verifyChecksum(cmd)) {
            ESP_LOGW(TAG, "Command checksum error!");
            return;
        }
        
        switch (cmd.type) {
            case CMD_ARM_DISARM:
                if (cmd.data.arm_disarm.arm) {
                    fc.arm();
                } else {
                    fc.disarm();
                }
                break;
                
            case CMD_TAKEOFF:
                fc.takeoff(cmd.data.takeoff.altitude);
                break;
                
            case CMD_LAND:
                fc.land();
                break;
                
            case CMD_SET_MODE:
                fc.setMode((FlightMode)cmd.data.set_mode.mode);
                break;
                
            case CMD_GOTO:
                fc.goto_position(cmd.data.goto_pos.x, 
                               cmd.data.goto_pos.y,
                               cmd.data.goto_pos.z,
                               cmd.data.goto_pos.yaw);
                break;
                
            case CMD_RTL:
                fc.returnToLaunch();
                break;
                
            case CMD_EMERGENCY_STOP:
                fc.emergencyStop();
                break;
        }
        
        // ACK送信
        sendAcknowledgment(cmd.sequence);
    }
    
private:
    bool verifyChecksum(const CommandPacket& cmd) {
        uint16_t calculated = calculateChecksum((uint8_t*)&cmd, 
                                               sizeof(cmd) - 2);
        return calculated == cmd.checksum;
    }
    
    uint16_t calculateChecksum(uint8_t* data, size_t length) {
        uint16_t sum = 0;
        for (size_t i = 0; i < length; i++) {
            sum += data[i];
        }
        return sum;
    }
};
```

### 5.8.3 高度ESP-NOW通信システム

要件定義書に基づいた高信頼性・低遅延通信の実装：

```c
// 高度ESP-NOW通信管理
#include "esp_crc.h"
#include "esp_system.h"

// 通信品質監視
typedef struct {
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t crc_errors;
    uint32_t timeout_errors;
    float packet_loss_rate;
    float round_trip_time_ms;
    int8_t rssi;
    uint32_t last_heartbeat;
} comm_stats_t;

static comm_stats_t g_comm_stats = {0};

// 拡張メッセージ構造体
typedef struct {
    uint8_t msg_type;
    uint8_t sequence_id;
    uint16_t crc16;
    uint32_t timestamp;
    uint8_t priority;       // 0:低 1:通常 2:高 3:緊急
    uint8_t flags;          // ACK要求、重複検出等
    union {
        struct {
            float roll, pitch, yaw, throttle;
            uint8_t arm_switch;
            uint8_t flight_mode;
            uint8_t safety_override;
            float target_position[3];
        } control_data;
        struct {
            float position[3];
            float velocity[3];
            float attitude[3];
            float angular_rate[3];
            float battery_voltage;
            float battery_current;
            uint8_t flight_status;
            uint8_t safety_status;
            uint32_t active_safety_events;
        } telemetry_data;
        struct {
            uint8_t system_status;
            uint8_t sensor_health;
            uint32_t flight_time;
            uint32_t error_codes;
        } status_data;
        struct {
            uint8_t original_sequence;
            uint8_t ack_status;
        } ack_data;
    };
} enhanced_esp_now_message_t;

// メッセージタイプ拡張
#define MSG_CONTROL_ENHANCED    0x11
#define MSG_TELEMETRY_ENHANCED  0x12
#define MSG_STATUS_ENHANCED     0x13
#define MSG_HEARTBEAT          0x14
#define MSG_ACK                0x15
#define MSG_EMERGENCY          0xFF

// フラグ定義
#define MSG_FLAG_ACK_REQUIRED   0x01
#define MSG_FLAG_ENCRYPTED      0x02
#define MSG_FLAG_COMPRESSED     0x04
#define MSG_FLAG_RETRANSMIT     0x08

static uint8_t g_sequence_counter = 0;
static QueueHandle_t tx_queue;
static QueueHandle_t rx_queue;

// 通信タスク
void esp_now_communication_task(void *parameters) {
    enhanced_esp_now_message_t tx_msg, rx_msg;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(10);  // 100Hz
    
    while (1) {
        // 送信キューの処理
        while (xQueueReceive(tx_queue, &tx_msg, 0) == pdTRUE) {
            send_enhanced_message(&tx_msg);
        }
        
        // 受信キューの処理
        while (xQueueReceive(rx_queue, &rx_msg, 0) == pdTRUE) {
            process_enhanced_message(&rx_msg);
        }
        
        // 通信品質監視
        update_communication_stats();
        
        // ハートビート送信（1Hz）
        static uint32_t heartbeat_counter = 0;
        if (++heartbeat_counter >= 100) {
            send_heartbeat();
            heartbeat_counter = 0;
        }
        
        vTaskDelayUntil(&last_wake_time, task_period);
    }
}

esp_err_t send_enhanced_message(enhanced_esp_now_message_t* msg) {
    // シーケンス番号設定
    msg->sequence_id = g_sequence_counter++;
    
    // タイムスタンプ設定
    msg->timestamp = esp_timer_get_time() / 1000;  // ms
    
    // CRC計算
    msg->crc16 = 0;  // CRC計算前は0
    msg->crc16 = esp_crc16_le(0, (uint8_t*)msg, sizeof(enhanced_esp_now_message_t) - 2);
    
    // 優先度に応じた送信処理
    esp_err_t result;
    switch (msg->priority) {
        case 3:  // 緊急
            result = esp_now_send(controller_mac, (uint8_t*)msg, sizeof(enhanced_esp_now_message_t));
            if (result != ESP_OK) {
                // 緊急メッセージは再送
                vTaskDelay(pdMS_TO_TICKS(1));
                result = esp_now_send(controller_mac, (uint8_t*)msg, sizeof(enhanced_esp_now_message_t));
            }
            break;
            
        case 2:  // 高優先度
        case 1:  // 通常
        default:
            result = esp_now_send(controller_mac, (uint8_t*)msg, sizeof(enhanced_esp_now_message_t));
            break;
    }
    
    // 統計更新
    g_comm_stats.packets_sent++;
    if (result != ESP_OK) {
        g_comm_stats.packets_lost++;
        ESP_LOGW("ESPNOW", "Send failed: %s", esp_err_to_name(result));
    }
    
    return result;
}

void process_enhanced_message(enhanced_esp_now_message_t* msg) {
    // CRC検証
    uint16_t received_crc = msg->crc16;
    msg->crc16 = 0;
    uint16_t calculated_crc = esp_crc16_le(0, (uint8_t*)msg, sizeof(enhanced_esp_now_message_t) - 2);
    
    if (received_crc != calculated_crc) {
        g_comm_stats.crc_errors++;
        ESP_LOGW("ESPNOW", "CRC error: received=0x%04X, calculated=0x%04X", 
                received_crc, calculated_crc);
        return;
    }
    
    // 重複検出（簡易実装）
    static uint8_t last_sequence_id = 0xFF;
    if (msg->sequence_id == last_sequence_id) {
        ESP_LOGD("ESPNOW", "Duplicate message detected: seq=%d", msg->sequence_id);
        return;
    }
    last_sequence_id = msg->sequence_id;
    
    // 通信タイムスタンプ更新
    g_safety_state.last_comm_time = esp_timer_get_time();
    g_comm_stats.packets_received++;
    
    // ACK要求処理
    if (msg->flags & MSG_FLAG_ACK_REQUIRED) {
        send_acknowledgment(msg->sequence_id);
    }
    
    // メッセージタイプ別処理
    switch (msg->msg_type) {
        case MSG_CONTROL_ENHANCED:
            process_control_command_enhanced(&msg->control_data);
            break;
            
        case MSG_STATUS_ENHANCED:
            ESP_LOGI("ESPNOW", "Enhanced status received");
            break;
            
        case MSG_HEARTBEAT:
            g_comm_stats.last_heartbeat = esp_timer_get_time() / 1000;
            ESP_LOGD("ESPNOW", "Heartbeat received");
            break;
            
        case MSG_ACK:
            process_acknowledgment(&msg->ack_data);
            break;
            
        case MSG_EMERGENCY:
            ESP_LOGE("ESPNOW", "EMERGENCY MESSAGE RECEIVED!");
            trigger_safety_event(SAFETY_EVENT_COMM_LOSS);  // 緊急停止
            break;
            
        default:
            ESP_LOGW("ESPNOW", "Unknown enhanced message type: 0x%02X", msg->msg_type);
            break;
    }
}

void send_acknowledgment(uint8_t original_sequence) {
    enhanced_esp_now_message_t ack_msg = {
        .msg_type = MSG_ACK,
        .priority = 2,  // 高優先度
        .flags = 0,
        .ack_data = {
            .original_sequence = original_sequence,
            .ack_status = 0x00  // OK
        }
    };
    
    send_enhanced_message(&ack_msg);
}

void send_heartbeat(void) {
    enhanced_esp_now_message_t heartbeat_msg = {
        .msg_type = MSG_HEARTBEAT,
        .priority = 1,  // 通常優先度
        .flags = 0
    };
    
    send_enhanced_message(&heartbeat_msg);
}

// 高度テレメトリ送信
esp_err_t send_enhanced_telemetry(const State& state) {
    enhanced_esp_now_message_t msg = {
        .msg_type = MSG_TELEMETRY_ENHANCED,
        .priority = 1,  // 通常優先度
        .flags = 0,
        .telemetry_data = {
            .position = {state.position.x, state.position.y, state.position.z},
            .velocity = {state.velocity.x, state.velocity.y, state.velocity.z},
            .attitude = {state.roll, state.pitch, state.yaw},
            .angular_rate = {state.gyro.x, state.gyro.y, state.gyro.z},
            .battery_voltage = g_battery_state.voltage,
            .battery_current = g_battery_state.current,
            .flight_status = get_flight_status(),
            .safety_status = get_safety_status(),
            .active_safety_events = g_safety_state.active_events
        }
    };
    
    return send_enhanced_message(&msg);
}

void update_communication_stats(void) {
    static uint32_t last_stats_time = 0;
    uint32_t current_time = esp_timer_get_time() / 1000;  // ms
    
    if (current_time - last_stats_time > 1000) {  // 1秒毎
        // パケットロス率計算
        if (g_comm_stats.packets_sent > 0) {
            g_comm_stats.packet_loss_rate = 
                (float)g_comm_stats.packets_lost / g_comm_stats.packets_sent * 100.0;
        }
        
        // 通信途絶検出
        if (current_time - g_comm_stats.last_heartbeat > 2000) {  // 2秒
            ESP_LOGW("ESPNOW", "Communication degraded - no heartbeat");
        }
        
        // 統計ログ出力（デバッグ時）
        ESP_LOGD("ESPNOW", "Stats: Sent=%lu, Recv=%lu, Lost=%lu, Loss=%.1f%%, CRC_Err=%lu",
                g_comm_stats.packets_sent, g_comm_stats.packets_received,
                g_comm_stats.packets_lost, g_comm_stats.packet_loss_rate,
                g_comm_stats.crc_errors);
        
        last_stats_time = current_time;
    }
}

// 通信品質取得
comm_stats_t get_communication_stats(void) {
    return g_comm_stats;
}

// ESP-NOW高度初期化
esp_err_t esp_now_enhanced_init(void) {
    // 基本ESP-NOW初期化
    esp_err_t ret = esp_now_init_drone();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // キュー作成
    tx_queue = xQueueCreate(20, sizeof(enhanced_esp_now_message_t));
    rx_queue = xQueueCreate(20, sizeof(enhanced_esp_now_message_t));
    
    if (!tx_queue || !rx_queue) {
        ESP_LOGE("ESPNOW", "Failed to create communication queues");
        return ESP_FAIL;
    }
    
    // 通信タスク開始
    xTaskCreate(esp_now_communication_task, "esp_now_comm", 4096, NULL, 5, NULL);
    
    ESP_LOGI("ESPNOW", "Enhanced ESP-NOW communication system initialized");
    return ESP_OK;
}
```

### 5.8.4 通信セキュリティ

```c
// ESP-NOW通信の暗号化とセキュリティ
#include "mbedtls/aes.h"
#include "esp_random.h"

#define AES_KEY_SIZE 16
#define AES_BLOCK_SIZE 16

static uint8_t aes_key[AES_KEY_SIZE] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

// 暗号化メッセージ送信
esp_err_t send_encrypted_message(enhanced_esp_now_message_t* msg) {
    if (!(msg->flags & MSG_FLAG_ENCRYPTED)) {
        return send_enhanced_message(msg);  // 暗号化不要
    }
    
    mbedtls_aes_context aes_ctx;
    mbedtls_aes_init(&aes_ctx);
    
    // AESキー設定
    int ret = mbedtls_aes_setkey_enc(&aes_ctx, aes_key, AES_KEY_SIZE * 8);
    if (ret != 0) {
        ESP_LOGE("CRYPTO", "AES key setup failed: %d", ret);
        mbedtls_aes_free(&aes_ctx);
        return ESP_FAIL;
    }
    
    // メッセージのペイロード部分を暗号化
    uint8_t* payload = (uint8_t*)&msg->control_data;
    size_t payload_size = sizeof(enhanced_esp_now_message_t) - 
                         offsetof(enhanced_esp_now_message_t, control_data);
    
    // AES-ECBで暗号化（簡易実装）
    for (size_t i = 0; i < payload_size; i += AES_BLOCK_SIZE) {
        size_t block_size = (payload_size - i < AES_BLOCK_SIZE) ? 
                           payload_size - i : AES_BLOCK_SIZE;
        
        if (block_size == AES_BLOCK_SIZE) {
            mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, 
                                 payload + i, payload + i);
        }
    }
    
    mbedtls_aes_free(&aes_ctx);
    
    // 暗号化済みメッセージ送信
    return send_enhanced_message(msg);
}

// 通信認証
bool authenticate_peer(const uint8_t* mac_addr) {
    // 事前登録されたMACアドレスリスト
    static const uint8_t authorized_macs[][6] = {
        {0x24, 0x6F, 0x28, 0xB5, 0x2C, 0x10},  // コントローラ1
        {0x24, 0x6F, 0x28, 0xB5, 0x2C, 0x11},  // コントローラ2（バックアップ）
    };
    
    for (int i = 0; i < sizeof(authorized_macs) / 6; i++) {
        if (memcmp(mac_addr, authorized_macs[i], 6) == 0) {
            return true;
        }
    }
    
    ESP_LOGW("SECURITY", "Unauthorized peer: " MACSTR, MAC2STR(mac_addr));
    return false;
}
```

## 5.9 フライトモード

### 5.9.1 フライトモード管理

```cpp
class FlightModeManager {
private:
    FlightMode current_mode = DISARMED;
    FlightMode previous_mode = DISARMED;
    
    // 各モードのコントローラ
    StabilizeMode stabilize_mode;
    AltitudeHoldMode altitude_mode;
    PositionHoldMode position_mode;
    AcroMode acro_mode;
    AutoMode auto_mode;
    
public:
    void setMode(FlightMode new_mode) {
        // モード遷移の検証
        if (!canTransitionTo(new_mode)) {
            ESP_LOGW(TAG, "Invalid mode transition!");
            return;
        }
        
        // 前モードの終了処理
        exitMode(current_mode);
        
        previous_mode = current_mode;
        current_mode = new_mode;
        
        // 新モードの初期化
        enterMode(new_mode);
        
        ESP_LOGI(TAG, "Mode changed to: %s", getModeString(new_mode));
    }
    
    ControlOutputs update(const State& state, const RCInput& rc_input) {
        switch (current_mode) {
            case STABILIZE:
                return stabilize_mode.update(state, rc_input);
                
            case ALTITUDE_HOLD:
                return altitude_mode.update(state, rc_input);
                
            case POSITION_HOLD:
                return position_mode.update(state, rc_input);
                
            case ACRO:
                return acro_mode.update(state, rc_input);
                
            case AUTO:
                return auto_mode.update(state);
                
            default:
                return ControlOutputs();  // ゼロ出力
        }
    }
    
private:
    bool canTransitionTo(FlightMode mode) {
        // 安全なモード遷移のチェック
        if (current_mode == DISARMED) {
            return mode == ARMED;
        }
        
        if (mode == DISARMED) {
            return true;  // いつでもディスアーム可能
        }
        
        // 飛行中のモード変更
        if (isFlying()) {
            // アクロモードへの遷移は高度が十分な場合のみ
            if (mode == ACRO && getAltitude() < 5.0) {
                return false;
            }
        }
        
        return true;
    }
};
```

### 5.9.2 各フライトモードの実装

```cpp
// スタビライズモード（自己水平）
class StabilizeMode {
private:
    AttitudeController attitude_ctrl;
    float max_angle = 35 * DEG_TO_RAD;
    
public:
    ControlOutputs update(const State& state, const RCInput& rc) {
        // RCスティックを目標姿勢角に変換
        float roll_target = rc.roll * max_angle;
        float pitch_target = rc.pitch * max_angle;
        float yaw_rate_target = rc.yaw * 180 * DEG_TO_RAD;  // deg/s
        
        // 姿勢制御
        Quaternion target_attitude = eulerToQuaternion(
            roll_target, pitch_target, state.yaw);
        Vector3 rate_cmd = attitude_ctrl.compute(state, target_attitude);
        
        // ヨーは角速度制御
        rate_cmd.z = yaw_rate_target;
        
        // 出力
        ControlOutputs output;
        output.moments = rate_cmd;
        output.thrust = rc.throttle;
        
        return output;
    }
};

// 高度維持モード
class AltitudeHoldMode : public StabilizeMode {
private:
    PIDController altitude_pid;
    float target_altitude;
    bool altitude_locked = false;
    
public:
    AltitudeHoldMode() : altitude_pid(2, 0.5, 0.1, 0.3, 0.3) {}
    
    ControlOutputs update(const State& state, const RCInput& rc) {
        // スタビライズモードの姿勢制御を継承
        ControlOutputs output = StabilizeMode::update(state, rc);
        
        // スロットル中立で高度ロック
        if (!altitude_locked && fabs(rc.throttle - 0.5) < 0.05) {
            target_altitude = state.position.z;
            altitude_locked = true;
        }
        
        if (altitude_locked) {
            // 高度制御
            float altitude_error = target_altitude - state.position.z;
            float thrust_adjustment = altitude_pid.update(
                altitude_error, 0, 0.01);
            
            output.thrust = HOVER_THRUST + thrust_adjustment;
            
            // スロットルスティックで目標高度を調整
            if (fabs(rc.throttle - 0.5) > 0.1) {
                target_altitude += (rc.throttle - 0.5) * 0.02;
                target_altitude = constrain(target_altitude, 0, 50);
            }
        }
        
        return output;
    }
};

// 位置保持モード
class PositionHoldMode {
private:
    PositionController pos_ctrl;
    VelocityController vel_ctrl;
    AttitudeController att_ctrl;
    Vector3 hold_position;
    bool position_locked = false;
    
public:
    ControlOutputs update(const State& state, const RCInput& rc) {
        // スティック中立で位置ロック
        if (!position_locked && 
            fabs(rc.roll) < 0.05 && fabs(rc.pitch) < 0.05) {
            hold_position = state.position;
            position_locked = true;
        }
        
        Vector3 velocity_cmd;
        
        if (position_locked) {
            // 位置制御
            velocity_cmd = pos_ctrl.compute(state, hold_position);
            
            // スティック入力で位置を調整
            if (fabs(rc.roll) > 0.1 || fabs(rc.pitch) > 0.1) {
                hold_position.x += rc.pitch * 0.05;
                hold_position.y += rc.roll * 0.05;
            }
        } else {
            // 速度制御モード
            velocity_cmd.x = rc.pitch * 5.0;  // m/s
            velocity_cmd.y = rc.roll * 5.0;
            velocity_cmd.z = (rc.throttle - 0.5) * 2.0;
        }
        
        // 速度→姿勢変換
        AttitudeSetpoint att_sp = vel_ctrl.compute(state, velocity_cmd);
        
        // 姿勢制御
        Vector3 rate_cmd = att_ctrl.compute(state, att_sp.quaternion);
        
        ControlOutputs output;
        output.moments = rate_cmd;
        output.thrust = att_sp.thrust;
        
        return output;
    }
};
```

## 5.10 自動飛行機能

### 5.10.1 自動離着陸

```cpp
class AutoTakeoffLand {
private:
    enum Phase {
        IDLE,
        SPINNING_UP,
        LIFTING_OFF,
        CLIMBING,
        HOVERING,
        DESCENDING,
        TOUCHDOWN,
        SPINNING_DOWN
    } phase = IDLE;
    
    float target_altitude = 2.0;  // m
    float climb_rate = 0.5;       // m/s
    float descend_rate = 0.3;     // m/s
    uint32_t phase_start_time;
    
public:
    void startTakeoff(float altitude) {
        target_altitude = altitude;
        phase = SPINNING_UP;
        phase_start_time = millis();
    }
    
    void startLanding() {
        phase = DESCENDING;
        phase_start_time = millis();
    }
    
    ControlOutputs update(const State& state) {
        ControlOutputs output;
        
        switch (phase) {
            case SPINNING_UP:
                // モータ始動
                output.thrust = MIN_THROTTLE + 0.1;
                if (millis() - phase_start_time > 2000) {
                    phase = LIFTING_OFF;
                }
                break;
                
            case LIFTING_OFF:
                // 離陸推力
                output.thrust = HOVER_THRUST * 1.2;
                if (state.position.z > 0.3) {
                    phase = CLIMBING;
                }
                break;
                
            case CLIMBING:
                // 上昇制御
                {
                    float altitude_error = target_altitude - state.position.z;
                    float velocity_error = climb_rate - state.velocity.z;
                    output.thrust = HOVER_THRUST + velocity_error * 0.3;
                    
                    if (altitude_error < 0.1) {
                        phase = HOVERING;
                        phase_start_time = millis();
                    }
                }
                break;
                
            case HOVERING:
                // ホバリング
                output.thrust = HOVER_THRUST;
                // 高度維持制御を追加
                break;
                
            case DESCENDING:
                // 降下制御
                {
                    float velocity_error = -descend_rate - state.velocity.z;
                    output.thrust = HOVER_THRUST + velocity_error * 0.3;
                    
                    // 地面検出
                    if (state.position.z < 0.3) {
                        phase = TOUCHDOWN;
                        phase_start_time = millis();
                    }
                }
                break;
                
            case TOUCHDOWN:
                // 着地
                output.thrust = MIN_THROTTLE + 0.05;
                if (millis() - phase_start_time > 1000) {
                    phase = SPINNING_DOWN;
                }
                break;
                
            case SPINNING_DOWN:
                // モータ停止
                output.thrust = 0;
                phase = IDLE;
                break;
        }
        
        // 姿勢は常に水平を維持
        output.moments = Vector3(
            -state.roll * 5,
            -state.pitch * 5,
            -state.gyro.z * 2
        );
        
        return output;
    }
};
```

### 5.10.2 ウェイポイントナビゲーション

```cpp
class WaypointNavigator {
private:
    struct Mission {
        vector<Waypoint> waypoints;
        int current_index = 0;
        bool is_active = false;
        bool auto_land = true;
    };
    
    Mission mission;
    PathFollower path_follower;
    float acceptance_radius = 1.0;  // m
    
public:
    void loadMission(const vector<Waypoint>& waypoints) {
        mission.waypoints = waypoints;
        mission.current_index = 0;
        mission.is_active = false;
    }
    
    void startMission() {
        if (mission.waypoints.empty()) {
            ESP_LOGW(TAG, "No mission loaded!");
            return;
        }
        
        mission.is_active = true;
        mission.current_index = 0;
        ESP_LOGI(TAG, "Mission started!");
    }
    
    Vector3 update(const State& state) {
        if (!mission.is_active || mission.waypoints.empty()) {
            return Vector3(0, 0, 0);
        }
        
        Waypoint& current_wp = mission.waypoints[mission.current_index];
        
        // 現在のウェイポイントへの距離
        Vector3 to_waypoint = current_wp.position - state.position;
        float distance = to_waypoint.magnitude();
        
        // ウェイポイント到達判定
        if (distance < acceptance_radius) {
            ESP_LOGI(TAG, "Reached waypoint %d", mission.current_index);
            
            mission.current_index++;
            
            // ミッション完了チェック
            if (mission.current_index >= mission.waypoints.size()) {
                mission.is_active = false;
                ESP_LOGI(TAG, "Mission completed!");
                
                if (mission.auto_land) {
                    // 自動着陸を開始
                    return Vector3(0, 0, -0.3);  // 降下
                }
                
                return Vector3(0, 0, 0);
            }
        }
        
        // 速度ベクトルの計算
        float desired_speed = current_wp.velocity;
        Vector3 velocity_cmd = to_waypoint.normalized() * desired_speed;
        
        // 高度は個別に制御
        velocity_cmd.z = (current_wp.position.z - state.position.z) * 2;
        velocity_cmd.z = constrain(velocity_cmd.z, -1, 1);
        
        return velocity_cmd;
    }
    
    void abortMission() {
        mission.is_active = false;
        ESP_LOGI(TAG, "Mission aborted!");
    }
};
```

### 5.10.3 Return to Launch (RTL)

```cpp
class ReturnToLaunch {
private:
    enum RTLPhase {
        CLIMB,
        RETURN,
        DESCEND,
        LAND
    } phase = CLIMB;
    
    Vector3 home_position;
    float rtl_altitude = 15.0;  // m
    float return_speed = 3.0;   // m/s
    
public:
    void setHome(const Vector3& position) {
        home_position = position;
        ESP_LOGI(TAG, "Home set: %.2f, %.2f, %.2f", 
                home_position.x, home_position.y, home_position.z);
    }
    
    void activate() {
        phase = CLIMB;
        ESP_LOGI(TAG, "RTL activated!");
    }
    
    Vector3 update(const State& state) {
        Vector3 velocity_cmd;
        
        switch (phase) {
            case CLIMB:
                // RTL高度まで上昇
                if (state.position.z < rtl_altitude) {
                    velocity_cmd.z = 1.0;  // m/s
                } else {
                    phase = RETURN;
                }
                break;
                
            case RETURN:
                // ホームポジションへ帰還
                {
                    Vector3 to_home = home_position - state.position;
                    to_home.z = 0;  // 水平距離のみ
                    float distance = to_home.magnitude();
                    
                    if (distance > 2.0) {
                        velocity_cmd = to_home.normalized() * return_speed;
                        velocity_cmd.z = 0;  // 高度維持
                    } else {
                        phase = DESCEND;
                    }
                }
                break;
                
            case DESCEND:
                // ホーム上空で降下
                velocity_cmd.x = home_position.x - state.position.x;
                velocity_cmd.y = home_position.y - state.position.y;
                velocity_cmd.z = -0.5;  // m/s
                
                if (state.position.z < 2.0) {
                    phase = LAND;
                }
                break;
                
            case LAND:
                // 着陸
                velocity_cmd = Vector3(0, 0, -0.2);
                break;
        }
        
        return velocity_cmd;
    }
};
```

## まとめ

本章では、M5StampFlyで実際に動作するマルチコプタ制御システムの実装について詳しく解説しました。主な内容：

1. **ハードウェア構成**：M5StampS3（ESP32-S3）、BMI270 IMU、BMM150磁力計、BMP280気圧センサ、VL53L3CX距離センサ、PMW3901MBオプティカルフロー
2. **階層的制御構造**：角速度→姿勢→速度→位置の多重ループ
3. **センサフュージョン**：相補フィルタ、Madgwickフィルタ、拡張カルマンフィルタ
4. **PID制御の実装**：各制御ループでの調整
5. **モータミキシング**：716-17600kvモータへの制御出力変換
6. **安全機能**：フェイルセーフ、緊急着陸、バッテリー監視（300mAh 1S高電圧）
7. **ESP-NOW通信システム**：低遅延テレメトリ、コマンド処理
8. **フライトモード**：手動から自動まで多様なモード
9. **自動飛行**：離着陸、ウェイポイント、RTL

重要なポイント：

- **FreeRTOSタスク管理**：リアルタイム性を保証する適切なタスク設計
- **ESP-NOW通信**：低遅延通信による高精度制御の実現
- **安全性**：常に安全を最優先に設計
- **モジュール性**：機能を分離して開発・テスト
- **ESP-IDF活用**：ネイティブな組み込み開発環境の利用

次章では、これらの実装をシミュレーションと実機でテストする方法を解説します。

## 参考リンク

### 技術資料
- [M5StampFly GitHub](https://github.com/kouhei1970/M5StampFly)
- [ESP-IDF プログラミングガイド](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [ESP-NOW API リファレンス](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/network/esp_now.html)
- [FreeRTOS カーネルガイド](https://www.freertos.org/Documentation/RTOS_book.html)
- [PID制御の調整方法](https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html)

### 実践的発表資料
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343) - 102.2K views
- [StampFlyで学ぶマルチコプタ制御](https://www.docswell.com/s/Kouhei_Ito/K38V1P-2024-02-10-094123) - 314.3K views
- [2025StampFly勉強会](https://www.docswell.com/s/Kouhei_Ito/K4VR7G-2025-03-23-104258) - 57.3K views
- [StampFly_Seminar資料](https://www.docswell.com/s/Kouhei_Ito/K228YN-2024-10-27-074715) - 16.1K views

---

*本記事は「StampFly制御システム完全ガイド」シリーズの第5章です。*