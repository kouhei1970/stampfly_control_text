# 第6章 飛行試験 - StampFly制御システム入門

## はじめに

前章まで、マルチコプタの理論から実装まで詳しく学んできました。本章では、開発した制御システムを実際にテストする方法を解説します。シミュレーションから始めて実機試験まで、安全で効率的な試験プロセスを4コマ分の内容として体系的に説明します。

## 6.1 シミュレーション環境の構築

### 6.1.1 Physics-based シミュレータ

実機試験の前に、安全で費用効果の高いシミュレーション環境を構築します。

```c
// ESP-IDFのインクルード
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

static const char* TAG = "FLIGHT_TEST";
```

```cpp
class DroneSimulator {
private:
    // 物理モデル
    struct PhysicsModel {
        float mass = 0.8;           // kg
        float arm_length = 0.1;     // m
        float Ixx = 0.01;           // kg⋅m²
        float Iyy = 0.01;           // kg⋅m²
        float Izz = 0.02;           // kg⋅m²
        float drag_coefficient = 0.1;
        float thrust_coefficient = 1e-5;
        float torque_coefficient = 1e-7;
    } physics;
    
    // 状態変数
    State drone_state;
    
    // 環境パラメータ
    struct Environment {
        Vector3 gravity = {0, 0, 9.81};
        float air_density = 1.225;     // kg/m³
        Vector3 wind_velocity = {0, 0, 0};
        float ground_level = 0;
    } environment;
    
    // シミュレーション設定
    float simulation_dt = 0.001;      // 1ms
    uint32_t simulation_time = 0;     // ms
    
public:
    void initialize() {
        // 初期状態の設定
        drone_state.position = Vector3(0, 0, 0);
        drone_state.velocity = Vector3(0, 0, 0);
        drone_state.attitude = Quaternion(1, 0, 0, 0);  // Identity
        drone_state.angular_velocity = Vector3(0, 0, 0);
        
        ESP_LOGI("SIMULATOR", "Drone simulator initialized");
    }
    
    void update(const MotorCommands& motor_cmd) {
        // モータ指令から力とモーメントを計算
        Forces forces = calculateForces(motor_cmd);
        
        // 外乱の追加
        addDisturbances(forces);
        
        // 物理演算
        updatePhysics(forces);
        
        // センサ測定値の生成
        updateSensorMeasurements();
        
        simulation_time += simulation_dt * 1000;
    }
    
    State getState() const { return drone_state; }
    
private:
    struct Forces {
        float total_thrust;
        Vector3 moments;        // [roll, pitch, yaw]
        Vector3 drag_force;
    };
    
    Forces calculateForces(const MotorCommands& cmd) {
        Forces forces;
        
        // 各モータの推力計算
        float motor_thrusts[4];
        for (int i = 0; i < 4; i++) {
            // PWM指令を推力に変換
            float normalized_cmd = (cmd.motors[i] - MIN_THROTTLE) / 
                                 (MAX_THROTTLE - MIN_THROTTLE);
            motor_thrusts[i] = physics.thrust_coefficient * 
                              pow(normalized_cmd, 2);
        }
        
        // 総推力
        forces.total_thrust = motor_thrusts[0] + motor_thrusts[1] + 
                             motor_thrusts[2] + motor_thrusts[3];
        
        // モーメント計算（X配置）
        forces.moments.x = physics.arm_length * 
                          (motor_thrusts[2] + motor_thrusts[1] - 
                           motor_thrusts[0] - motor_thrusts[3]) / sqrt(2);
        forces.moments.y = physics.arm_length * 
                          (motor_thrusts[0] + motor_thrusts[1] - 
                           motor_thrusts[2] - motor_thrusts[3]) / sqrt(2);
        
        // ヨーモーメント（反トルク）
        forces.moments.z = physics.torque_coefficient * 
                          (motor_thrusts[0] - motor_thrusts[1] + 
                           motor_thrusts[2] - motor_thrusts[3]);
        
        // 空気抵抗
        Vector3 relative_velocity = drone_state.velocity - environment.wind_velocity;
        float velocity_magnitude = relative_velocity.magnitude();
        forces.drag_force = relative_velocity * (-physics.drag_coefficient * 
                                               velocity_magnitude);
        
        return forces;
    }
    
    void updatePhysics(const Forces& forces) {
        // 並進運動（地球座標系）
        Vector3 thrust_earth = drone_state.attitude.rotateVector(
            Vector3(0, 0, -forces.total_thrust));
        Vector3 total_force = thrust_earth + 
                             Vector3(0, 0, physics.mass * environment.gravity.z) +
                             forces.drag_force;
        
        Vector3 acceleration = total_force / physics.mass;
        
        // 位置と速度の更新（Verlet積分）
        Vector3 new_velocity = drone_state.velocity + acceleration * simulation_dt;
        Vector3 new_position = drone_state.position + 
                              (drone_state.velocity + new_velocity) * 0.5 * simulation_dt;
        
        // 地面衝突検出
        if (new_position.z <= environment.ground_level) {
            new_position.z = environment.ground_level;
            new_velocity.z = 0;
            // エネルギー散逸
            new_velocity *= 0.5;
        }
        
        drone_state.position = new_position;
        drone_state.velocity = new_velocity;
        
        // 回転運動（機体座標系）
        Vector3 angular_acceleration;
        angular_acceleration.x = (forces.moments.x - 
                                drone_state.angular_velocity.y * 
                                drone_state.angular_velocity.z * 
                                (physics.Izz - physics.Iyy)) / physics.Ixx;
        angular_acceleration.y = (forces.moments.y - 
                                drone_state.angular_velocity.z * 
                                drone_state.angular_velocity.x * 
                                (physics.Ixx - physics.Izz)) / physics.Iyy;
        angular_acceleration.z = (forces.moments.z - 
                                drone_state.angular_velocity.x * 
                                drone_state.angular_velocity.y * 
                                (physics.Iyy - physics.Ixx)) / physics.Izz;
        
        // 角速度の更新
        drone_state.angular_velocity += angular_acceleration * simulation_dt;
        
        // 四元数の更新
        Quaternion q_dot = drone_state.attitude.derivative(drone_state.angular_velocity);
        drone_state.attitude += q_dot * simulation_dt;
        drone_state.attitude.normalize();
    }
    
    void addDisturbances(Forces& forces) {
        // 風外乱
        static float wind_time = 0;
        wind_time += simulation_dt;
        
        // 簡単な乱流モデル
        float turbulence_x = 0.5 * sin(wind_time * 0.1) + 
                           0.2 * sin(wind_time * 0.7);
        float turbulence_y = 0.3 * cos(wind_time * 0.15) + 
                           0.1 * cos(wind_time * 0.9);
        
        environment.wind_velocity.x = turbulence_x;
        environment.wind_velocity.y = turbulence_y;
        
        // モータの不確かさ
        static uint32_t last_disturbance = 0;
        if (simulation_time - last_disturbance > 100) {  // 100ms毎
            forces.total_thrust *= (1.0 + (random(-50, 50) / 1000.0));
            forces.moments.x *= (1.0 + (random(-30, 30) / 1000.0));
            forces.moments.y *= (1.0 + (random(-30, 30) / 1000.0));
            last_disturbance = simulation_time;
        }
    }
    
    void updateSensorMeasurements() {
        // IMU測定値の生成（ノイズ付き）
        Vector3 gravity_body = drone_state.attitude.rotateVectorInverse(
            environment.gravity);
        Vector3 acceleration_body = drone_state.attitude.rotateVectorInverse(
            drone_state.velocity) + gravity_body;  // 簡略化
        
        // ノイズ追加
        acceleration_body.x += random(-100, 100) / 10000.0;  // ±0.01 m/s²
        acceleration_body.y += random(-100, 100) / 10000.0;
        acceleration_body.z += random(-100, 100) / 10000.0;
        
        drone_state.imu.acceleration = acceleration_body;
        
        // ジャイロノイズ
        drone_state.imu.angular_velocity = drone_state.angular_velocity;
        drone_state.imu.angular_velocity.x += random(-10, 10) / 10000.0;  // ±0.001 rad/s
        drone_state.imu.angular_velocity.y += random(-10, 10) / 10000.0;
        drone_state.imu.angular_velocity.z += random(-10, 10) / 10000.0;
    }
};
```

### 6.1.2 Hardware-in-the-Loop (HIL) シミュレーション

実際のM5StampFlyハードウェアを使ったシミュレーション：

```cpp
class HILSimulation {
private:
    DroneSimulator simulator;
    FlightController flight_controller;
    
    // 通信設定
    uart_port_t hil_uart = UART_NUM_1;
    
    struct HILMessage {
        uint8_t header[2] = {'H', 'I'};
        uint32_t timestamp;
        float accelerometer[3];
        float gyroscope[3];
        float position[3];
        float attitude[4];  // quaternion
        uint16_t checksum;
    } __attribute__((packed));
    
public:
    void init_hil() {
        hil_serial->begin(115200);
        simulator.initialize();
        flight_controller.initialize();
        
        ESP_LOGI("HIL", "HIL Simulation started");
    }
    
    void hil_task() {
        static uint32_t last_update = micros();
        uint32_t now = micros();
        float dt = (now - last_update) * 1e-6;
        
        if (dt >= 0.001) {  // 1kHz
            // センサデータをシミュレータから取得
            State sim_state = simulator.getState();
            
            // HILメッセージ作成
            HILMessage msg;
            msg.timestamp = millis();
            
            msg.accelerometer[0] = sim_state.imu.acceleration.x;
            msg.accelerometer[1] = sim_state.imu.acceleration.y;
            msg.accelerometer[2] = sim_state.imu.acceleration.z;
            
            msg.gyroscope[0] = sim_state.imu.angular_velocity.x;
            msg.gyroscope[1] = sim_state.imu.angular_velocity.y;
            msg.gyroscope[2] = sim_state.imu.angular_velocity.z;
            
            msg.position[0] = sim_state.position.x;
            msg.position[1] = sim_state.position.y;
            msg.position[2] = sim_state.position.z;
            
            msg.attitude[0] = sim_state.attitude.w;
            msg.attitude[1] = sim_state.attitude.x;
            msg.attitude[2] = sim_state.attitude.y;
            msg.attitude[3] = sim_state.attitude.z;
            
            msg.checksum = calculateChecksum((uint8_t*)&msg, sizeof(msg) - 2);
            
            // フライトコントローラに送信
            hil_serial->write((uint8_t*)&msg, sizeof(msg));
            
            // フライトコントローラからモータ指令を受信
            MotorCommands motor_cmd;
            if (receiveMotorCommands(motor_cmd)) {
                // シミュレータに適用
                simulator.update(motor_cmd);
            }
            
            last_update = now;
        }
    }
    
private:
    bool receiveMotorCommands(MotorCommands& cmd) {
        static uint8_t buffer[16];
        static int buffer_index = 0;
        
        while (hil_serial->available()) {
            buffer[buffer_index++] = hil_serial->read();
            
            if (buffer_index >= 16) {
                // モータ指令をパース
                memcpy(&cmd, buffer, sizeof(MotorCommands));
                buffer_index = 0;
                return true;
            }
        }
        return false;
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

### 6.1.3 自動テストフレームワーク

```cpp
class AutomatedTestFramework {
private:
    DroneSimulator simulator;
    FlightController controller;
    
    struct TestCase {
        String name;
        void (*setup_func)();
        bool (*test_func)(const State&);
        void (*cleanup_func)();
        float timeout_seconds;
        bool passed;
    };
    
    vector<TestCase> test_cases;
    
public:
    void addTest(const String& name, 
                void (*setup)(), 
                bool (*test)(const State&),
                void (*cleanup)(),
                float timeout = 30.0) {
        test_cases.push_back({name, setup, test, cleanup, timeout, false});
    }
    
    void runAllTests() {
        ESP_LOGI("TEST", "=== Automated Test Suite ===");
        
        int passed = 0;
        int total = test_cases.size();
        
        for (auto& test : test_cases) {
            ESP_LOGI("TEST", "Running test: %s", test.name.c_str());
            
            // テスト環境の初期化
            simulator.initialize();
            controller.reset();
            if (test.setup_func) test.setup_func();
            
            // テスト実行
            test.passed = runSingleTest(test);
            
            if (test.passed) {
                ESP_LOGI("TEST", "  PASSED");
                passed++;
            } else {
                ESP_LOGE("TEST", "  FAILED");
            }
            
            // クリーンアップ
            if (test.cleanup_func) test.cleanup_func();
        }
        
        ESP_LOGI("TEST", "Test Results: %d/%d tests passed", passed, total);
    }
    
private:
    bool runSingleTest(const TestCase& test) {
        uint32_t start_time = millis();
        uint32_t timeout_ms = test.timeout_seconds * 1000;
        
        while (millis() - start_time < timeout_ms) {
            // シミュレーション更新
            MotorCommands cmd = controller.update();
            simulator.update(cmd);
            State state = simulator.getState();
            
            // テスト条件チェック
            if (test.test_func(state)) {
                return true;
            }
            
            delay(1);  // 1ms
        }
        
        return false;  // タイムアウト
    }
};

// テストケースの例
void setupHoverTest() {
    // ホバリングテストの初期化
}

bool hoverTest(const State& state) {
    // ホバリングの安定性をチェック
    static uint32_t stable_start = 0;
    const float POSITION_TOLERANCE = 0.1;  // m
    const float VELOCITY_TOLERANCE = 0.1;  // m/s
    const uint32_t STABLE_DURATION = 5000; // ms
    
    float position_error = state.position.magnitude();
    float velocity_magnitude = state.velocity.magnitude();
    
    if (position_error < POSITION_TOLERANCE && 
        velocity_magnitude < VELOCITY_TOLERANCE) {
        if (stable_start == 0) {
            stable_start = millis();
        } else if (millis() - stable_start > STABLE_DURATION) {
            return true;  // テスト合格
        }
    } else {
        stable_start = 0;  // リセット
    }
    
    return false;
}
```

## 6.2 制御パラメータの調整

### 6.2.1 システム同定

実機の特性を測定して数学モデルを作成：

```cpp
class SystemIdentification {
private:
    struct DataPoint {
        float timestamp;
        float input;
        float output;
    };
    
    vector<DataPoint> data_log;
    
public:
    void startLogging() {
        data_log.clear();
        ESP_LOGI("SYSID", "System ID logging started");
    }
    
    void logData(float input, float output) {
        DataPoint point;
        point.timestamp = millis() * 0.001;  // seconds
        point.input = input;
        point.output = output;
        data_log.push_back(point);
    }
    
    void stopLogging() {
        ESP_LOGI("SYSID", "System ID logging stopped");
        ESP_LOGI("SYSID", "Collected %zu data points", data_log.size());
    }
    
    // ステップ応答による1次システム同定
    struct FirstOrderModel {
        float K;      // ゲイン
        float tau;    // 時定数
        float delay;  // 無駄時間
        float r_squared; // 決定係数
    };
    
    FirstOrderModel identifyFirstOrder() {
        FirstOrderModel model;
        
        if (data_log.size() < 100) {
            ESP_LOGW("SYSID", "Not enough data for identification");
            return model;
        }
        
        // ステップ入力の開始点を検出
        int step_start = findStepStart();
        if (step_start < 0) {
            ESP_LOGW("SYSID", "Step input not detected");
            return model;
        }
        
        // 定常値の推定
        float steady_state = calculateSteadyState(step_start + 100);
        
        // 63.2%値（時定数）の検出
        float target_value = steady_state * 0.632;
        int tau_index = findValueCrossing(step_start, target_value);
        
        if (tau_index > 0) {
            model.tau = data_log[tau_index].timestamp - 
                       data_log[step_start].timestamp;
            model.K = steady_state / getStepMagnitude(step_start);
        }
        
        // フィット品質の評価
        model.r_squared = calculateGoodnessOfFit(model);
        
        ESP_LOGI("SYSID", "Identified model: K=%.3f, tau=%.3f, R²=%.3f", 
                 model.K, model.tau, model.r_squared);
        
        return model;
    }
    
private:
    int findStepStart() {
        for (int i = 1; i < data_log.size(); i++) {
            float input_change = abs(data_log[i].input - data_log[i-1].input);
            if (input_change > 0.1) {  // 10%以上の変化
                return i;
            }
        }
        return -1;
    }
    
    float calculateSteadyState(int start_index) {
        if (start_index + 50 >= data_log.size()) {
            return data_log.back().output;
        }
        
        float sum = 0;
        int count = min(50, (int)data_log.size() - start_index);
        
        for (int i = 0; i < count; i++) {
            sum += data_log[start_index + i].output;
        }
        
        return sum / count;
    }
    
    float calculateGoodnessOfFit(const FirstOrderModel& model) {
        float ss_res = 0;  // 残差平方和
        float ss_tot = 0;  // 全平方和
        
        float mean_output = 0;
        for (const auto& point : data_log) {
            mean_output += point.output;
        }
        mean_output /= data_log.size();
        
        for (int i = 1; i < data_log.size(); i++) {
            float t = data_log[i].timestamp - data_log[0].timestamp;
            float predicted = model.K * (1 - exp(-t / model.tau));
            float actual = data_log[i].output;
            
            ss_res += pow(actual - predicted, 2);
            ss_tot += pow(actual - mean_output, 2);
        }
        
        return 1 - (ss_res / ss_tot);
    }
};
```

### 6.2.2 PIDチューニング

```cpp
class PIDTuner {
private:
    struct TuningResult {
        float kp, ki, kd;
        float performance_index;
        bool stable;
    };
    
    DroneSimulator simulator;
    SystemIdentification sysid;
    
public:
    // Ziegler-Nichols法
    TuningResult tuneZieglerNichols(float ultimate_gain, float ultimate_period) {
        TuningResult result;
        
        // PID係数の計算
        result.kp = 0.6 * ultimate_gain;
        result.ki = 1.2 * ultimate_gain / ultimate_period;
        result.kd = 0.075 * ultimate_gain * ultimate_period;
        
        // 性能評価
        result.performance_index = evaluatePerformance(result.kp, result.ki, result.kd);
        result.stable = checkStability(result.kp, result.ki, result.kd);
        
        ESP_LOGI("TUNE", "Ziegler-Nichols tuning result:");
        ESP_LOGI("TUNE", "  Kp: %.3f", result.kp);
        ESP_LOGI("TUNE", "  Ki: %.3f", result.ki);
        ESP_LOGI("TUNE", "  Kd: %.3f", result.kd);
        ESP_LOGI("TUNE", "  Performance: %.3f", result.performance_index);
        ESP_LOGI("TUNE", "  Stable: %s", result.stable ? "Yes" : "No");
        
        return result;
    }
    
    // 遺伝的アルゴリズムによる最適化
    TuningResult tuneGenetic(int population_size = 20, int generations = 50) {
        vector<TuningResult> population;
        
        // 初期集団の生成
        for (int i = 0; i < population_size; i++) {
            TuningResult individual;
            individual.kp = random(0, 200) / 10.0;    // 0-20
            individual.ki = random(0, 100) / 10.0;    // 0-10
            individual.kd = random(0, 50) / 10.0;     // 0-5
            individual.performance_index = evaluatePerformance(
                individual.kp, individual.ki, individual.kd);
            individual.stable = checkStability(
                individual.kp, individual.ki, individual.kd);
            population.push_back(individual);
        }
        
        // 進化ループ
        for (int gen = 0; gen < generations; gen++) {
            // 選択・交叉・突然変異
            evolvePopulation(population);
            
            // 進捗表示
            TuningResult best = getBest(population);
            ESP_LOGI("GA", "Generation %d: Best performance = %.3f", 
                     gen, best.performance_index);
        }
        
        return getBest(population);
    }
    
private:
    float evaluatePerformance(float kp, float ki, float kd) {
        // ステップ応答による性能評価
        simulator.initialize();
        
        PIDController pid(kp, ki, kd, 10, 100);
        
        float setpoint = 1.0;
        float integral_error = 0;
        float max_overshoot = 0;
        float settling_time = 0;
        bool settled = false;
        
        for (int t = 0; t < 5000; t++) {  // 5秒間
            float dt = 0.001;
            State state = simulator.getState();
            
            float control = pid.update(setpoint, state.position.z, dt);
            MotorCommands cmd;
            cmd.motors[0] = cmd.motors[1] = cmd.motors[2] = cmd.motors[3] = 
                HOVER_THROTTLE + control;
            
            simulator.update(cmd);
            
            // 性能指標の計算
            float error = setpoint - state.position.z;
            integral_error += abs(error) * dt;
            
            if (state.position.z > max_overshoot) {
                max_overshoot = state.position.z;
            }
            
            if (!settled && abs(error) < 0.02) {  // 2%以内
                settling_time = t * dt;
                settled = true;
            }
        }
        
        // 性能指標の計算（小さいほど良い）
        float overshoot_penalty = max(0.0f, max_overshoot - setpoint) * 10;
        float settling_penalty = settling_time;
        float error_penalty = integral_error;
        
        return 1.0 / (1.0 + overshoot_penalty + settling_penalty + error_penalty);
    }
    
    bool checkStability(float kp, float ki, float kd) {
        // ステップ応答の振動をチェック
        simulator.initialize();
        PIDController pid(kp, ki, kd, 10, 100);
        
        float prev_output = 0;
        int oscillation_count = 0;
        
        for (int t = 0; t < 3000; t++) {
            float dt = 0.001;
            State state = simulator.getState();
            
            float control = pid.update(1.0, state.position.z, dt);
            MotorCommands cmd;
            cmd.motors[0] = cmd.motors[1] = cmd.motors[2] = cmd.motors[3] = 
                HOVER_THROTTLE + control;
            
            simulator.update(cmd);
            
            // 振動検出
            if (t > 100) {  // 初期の過渡応答を除外
                if ((state.position.z > prev_output && prev_output > 0) ||
                    (state.position.z < prev_output && prev_output < 0)) {
                    oscillation_count++;
                }
            }
            
            prev_output = state.position.z;
        }
        
        return oscillation_count < 10;  // 振動が少ない
    }
    
    void evolvePopulation(vector<TuningResult>& population) {
        // トーナメント選択
        vector<TuningResult> new_population;
        
        for (int i = 0; i < population.size(); i++) {
            TuningResult parent1 = tournamentSelection(population);
            TuningResult parent2 = tournamentSelection(population);
            
            // 交叉
            TuningResult offspring = crossover(parent1, parent2);
            
            // 突然変異
            mutate(offspring);
            
            // 性能評価
            offspring.performance_index = evaluatePerformance(
                offspring.kp, offspring.ki, offspring.kd);
            offspring.stable = checkStability(
                offspring.kp, offspring.ki, offspring.kd);
            
            new_population.push_back(offspring);
        }
        
        population = new_population;
    }
    
    TuningResult tournamentSelection(const vector<TuningResult>& population) {
        int idx1 = random(0, population.size());
        int idx2 = random(0, population.size());
        
        if (population[idx1].performance_index > population[idx2].performance_index) {
            return population[idx1];
        } else {
            return population[idx2];
        }
    }
    
    TuningResult crossover(const TuningResult& parent1, const TuningResult& parent2) {
        TuningResult offspring;
        offspring.kp = (parent1.kp + parent2.kp) * 0.5;
        offspring.ki = (parent1.ki + parent2.ki) * 0.5;
        offspring.kd = (parent1.kd + parent2.kd) * 0.5;
        return offspring;
    }
    
    void mutate(TuningResult& individual) {
        float mutation_rate = 0.1;
        
        if (random(0, 100) < mutation_rate * 100) {
            individual.kp += random(-20, 20) / 100.0;
            individual.kp = max(0.0f, individual.kp);
        }
        
        if (random(0, 100) < mutation_rate * 100) {
            individual.ki += random(-10, 10) / 100.0;
            individual.ki = max(0.0f, individual.ki);
        }
        
        if (random(0, 100) < mutation_rate * 100) {
            individual.kd += random(-5, 5) / 100.0;
            individual.kd = max(0.0f, individual.kd);
        }
    }
    
    TuningResult getBest(const vector<TuningResult>& population) {
        TuningResult best = population[0];
        for (const auto& individual : population) {
            if (individual.performance_index > best.performance_index && 
                individual.stable) {
                best = individual;
            }
        }
        return best;
    }
};
```

### 6.2.3 オンラインモデル予測制御

```cpp
class ModelPredictiveController {
private:
    struct MPCParameters {
        int prediction_horizon = 10;
        int control_horizon = 5;
        float weight_state = 1.0f;
        float weight_control = 0.1f;
    } params;
    
    struct PerformanceMonitor {
        float error_history[100];
        int history_index = 0;
        float average_error = 0;
        float error_variance = 0;
    } monitor;
    
public:
    void updateGains(float error, float error_rate) {
        // エラー履歴の更新
        monitor.error_history[monitor.history_index] = abs(error);
        monitor.history_index = (monitor.history_index + 1) % 100;
        
        // 統計的指標の計算
        calculateStatistics();
        
        // ゲイン適応ルール
        if (monitor.error_variance > 0.1) {
            // 大きな外乱→ダンピングを増やす
            gains.kd += gains.adaptation_rate;
            gains.kp *= 0.99;
        } else if (monitor.average_error > 0.05) {
            // 定常偏差→積分ゲインを増やす
            gains.ki += gains.adaptation_rate * 0.1;
        } else if (abs(error_rate) > 0.1) {
            // 振動→比例ゲインを下げる
            gains.kp *= 0.99;
        }
        
        // ゲインの制限
        gains.kp = constrain(gains.kp, 0.1, 50);
        gains.ki = constrain(gains.ki, 0, 10);
        gains.kd = constrain(gains.kd, 0, 5);
    }
    
private:
    void calculateStatistics() {
        float sum = 0;
        for (int i = 0; i < 100; i++) {
            sum += monitor.error_history[i];
        }
        monitor.average_error = sum / 100;
        
        float variance_sum = 0;
        for (int i = 0; i < 100; i++) {
            float diff = monitor.error_history[i] - monitor.average_error;
            variance_sum += diff * diff;
        }
        monitor.error_variance = variance_sum / 100;
    }
};
```

## 6.3 実機試験の安全手順

### 6.3.1 プリフライトチェック

```cpp
class PreFlightInspection {
private:
    struct CheckItem {
        String description;
        bool (*check_function)();
        bool is_critical;
        bool status;
    };
    
    vector<CheckItem> checklist;
    
public:
    void initializeChecklist() {
        checklist = {
            {"Battery voltage check", checkBatteryVoltage, true, false},
            {"Motor response test", checkMotorResponse, true, false},
            {"IMU calibration status", checkIMUCalibration, true, false},
            {"Control surface movement", checkControlSurfaces, true, false},
            {"Communication link", checkCommLink, true, false},
            {"Propeller installation", checkPropellers, true, false},
            {"Frame integrity", checkFrameIntegrity, true, false},
            {"GPS fix (if available)", checkGPSFix, false, false},
            {"Weather conditions", checkWeatherConditions, false, false}
        };
    }
    
    bool runPreFlightCheck() {
        ESP_LOGI("PREFLIGHT", "=== Pre-Flight Inspection ===");
        
        bool all_critical_passed = true;
        bool all_non_critical_passed = true;
        
        for (auto& item : checklist) {
            ESP_LOGI("PREFLIGHT", "Checking: %s...", item.description.c_str());
            
            item.status = item.check_function();
            
            if (item.status) {
                ESP_LOGI("PREFLIGHT", "PASS");
            } else {
                ESP_LOGE("PREFLIGHT", "FAIL");
                if (item.is_critical) {
                    all_critical_passed = false;
                } else {
                    all_non_critical_passed = false;
                }
            }
        }
        
        // 結果判定
        if (!all_critical_passed) {
            ESP_LOGE("PREFLIGHT", "CRITICAL ITEMS FAILED - FLIGHT NOT AUTHORIZED");
            return false;
        } else if (!all_non_critical_passed) {
            ESP_LOGW("PREFLIGHT", "NON-CRITICAL ITEMS FAILED - PROCEED WITH CAUTION");
            return true;
        } else {
            ESP_LOGI("PREFLIGHT", "ALL CHECKS PASSED - FLIGHT AUTHORIZED");
            return true;
        }
    }
    
private:
    static bool checkBatteryVoltage() {
        float voltage = analogRead(BATTERY_PIN) * 3.3 / 4095.0 * 2.0;  // 分圧回路
        ESP_LOGI("PREFLIGHT", " (%.2fV) ", voltage);
        return voltage > 3.5;  // 1セルLiPoの最低電圧
    }
    
    static bool checkMotorResponse() {
        // 各モータの回転テスト
        int test_throttle = 1200;  // 安全な低出力
        
        for (int motor = 0; motor < 4; motor++) {
            // モータを短時間回転
            setMotorThrottle(motor, test_throttle);
            delay(500);
            setMotorThrottle(motor, 1000);
            delay(200);
            
            // 電流センサがあれば電流値をチェック
            // RPMセンサがあれば回転数をチェック
        }
        
        return true;  // 簡略化
    }
    
    static bool checkIMUCalibration() {
        // ジャイロのバイアスチェック
        float gyro_sum[3] = {0, 0, 0};
        int samples = 100;
        
        for (int i = 0; i < samples; i++) {
            float gx, gy, gz;
            readGyro(&gx, &gy, &gz);
            gyro_sum[0] += gx;
            gyro_sum[1] += gy;
            gyro_sum[2] += gz;
            delay(10);
        }
        
        float gyro_bias[3] = {
            gyro_sum[0] / samples,
            gyro_sum[1] / samples,
            gyro_sum[2] / samples
        };
        
        // バイアスが小さいかチェック
        float max_bias = max({abs(gyro_bias[0]), abs(gyro_bias[1]), abs(gyro_bias[2])});
        return max_bias < 0.05;  // 0.05 rad/s以下
    }
    
    static bool checkCommLink() {
        // 通信リンクの品質チェック
        static uint32_t last_packet = 0;
        uint32_t now = millis();
        
        // パケット受信の確認
        if (now - last_packet < 1000) {
            return true;
        } else {
            return false;
        }
    }
};
```

### 6.3.2 段階的テスト手順

```cpp
class IncrementalTesting {
private:
    enum TestPhase {
        GROUND_TEST,
        TETHERED_TEST,
        LOW_HOVER,
        MEDIUM_HOVER,
        FULL_FLIGHT
    } current_phase = GROUND_TEST;
    
    struct TestCriteria {
        float max_altitude;
        float max_velocity;
        float max_acceleration;
        float max_angular_rate;
        bool gps_required;
        uint32_t max_duration;  // seconds
    };
    
    map<TestPhase, TestCriteria> phase_limits = {
        {GROUND_TEST, {0, 0, 0, 0, false, 60}},
        {TETHERED_TEST, {0.5, 0.1, 0.5, 0.5, false, 300}},
        {LOW_HOVER, {2, 0.5, 1, 1, false, 600}},
        {MEDIUM_HOVER, {10, 2, 2, 2, false, 1800}},
        {FULL_FLIGHT, {50, 10, 5, 5, true, 3600}}
    };
    
    uint32_t phase_start_time = 0;
    
public:
    bool startPhase(TestPhase phase) {
        // 前段階の完了チェック
        if (phase > current_phase + 1) {
            ESP_LOGW("TEST", "Cannot skip test phases");
            return false;
        }
        
        current_phase = phase;
        phase_start_time = millis();
        
        ESP_LOGI("TEST", "Starting test phase: %s", getPhaseString(phase).c_str());
        
        // 制限値の設定
        TestCriteria limits = phase_limits[phase];
        setSafetyLimits(limits);
        
        return true;
    }
    
    bool checkLimits(const State& state) {
        TestCriteria limits = phase_limits[current_phase];
        
        // 高度制限
        if (state.position.z > limits.max_altitude) {
            ESP_LOGE("SAFETY", "Altitude limit exceeded!");
            return false;
        }
        
        // 速度制限
        float velocity = state.velocity.magnitude();
        if (velocity > limits.max_velocity) {
            ESP_LOGE("SAFETY", "Velocity limit exceeded!");
            return false;
        }
        
        // 角速度制限
        float angular_rate = state.angular_velocity.magnitude();
        if (angular_rate > limits.max_angular_rate) {
            ESP_LOGE("SAFETY", "Angular rate limit exceeded!");
            return false;
        }
        
        // 時間制限
        uint32_t elapsed = (millis() - phase_start_time) / 1000;
        if (elapsed > limits.max_duration) {
            ESP_LOGE("SAFETY", "Phase duration limit exceeded!");
            return false;
        }
        
        return true;
    }
    
    void completePhase() {
        ESP_LOGI("TEST", "Phase %s completed successfully", 
                 getPhaseString(current_phase).c_str());
        
        logPhaseCompletion();
    }
    
private:
    String getPhaseString(TestPhase phase) {
        switch (phase) {
            case GROUND_TEST: return "GROUND_TEST";
            case TETHERED_TEST: return "TETHERED_TEST";
            case LOW_HOVER: return "LOW_HOVER";
            case MEDIUM_HOVER: return "MEDIUM_HOVER";
            case FULL_FLIGHT: return "FULL_FLIGHT";
            default: return "UNKNOWN";
        }
    }
    
    void setSafetyLimits(const TestCriteria& limits) {
        // フライトコントローラに制限値を設定
        // これにより、制御システム自体が制限を強制
        flight_controller.setAltitudeLimit(limits.max_altitude);
        flight_controller.setVelocityLimit(limits.max_velocity);
        flight_controller.setAngularRateLimit(limits.max_angular_rate);
    }
    
    void logPhaseCompletion() {
        // テスト結果のログ保存
        String log_entry = String(millis()) + "," + 
                          getPhaseString(current_phase) + ",COMPLETED\n";
        
        // SDカードまたはフラッシュメモリに保存
        saveToLog(log_entry);
    }
};
```

### 6.3.3 緊急時手順

```cpp
class EmergencyProcedures {
private:
    enum EmergencyType {
        COMMUNICATION_LOSS,
        LOW_BATTERY,
        CONTROL_FAILURE,
        SENSOR_FAILURE,
        EXTERNAL_ABORT
    };
    
    struct EmergencyAction {
        EmergencyType type;
        void (*action_function)();
        uint32_t timeout_ms;
        bool executed;
    };
    
    vector<EmergencyAction> emergency_actions;
    bool emergency_active = false;
    
public:
    void initialize() {
        emergency_actions = {
            {COMMUNICATION_LOSS, handleCommLoss, 10000, false},
            {LOW_BATTERY, handleLowBattery, 30000, false},
            {CONTROL_FAILURE, handleControlFailure, 5000, false},
            {SENSOR_FAILURE, handleSensorFailure, 5000, false},
            {EXTERNAL_ABORT, handleExternalAbort, 1000, false}
        };
    }
    
    void checkEmergencyConditions(const State& state) {
        // 通信ロス検出
        static uint32_t last_comm = millis();
        if (millis() - last_comm > 3000) {  // 3秒間通信なし
            triggerEmergency(COMMUNICATION_LOSS);
        }
        
        // 低バッテリー検出
        if (state.battery_voltage < 3.3) {
            triggerEmergency(LOW_BATTERY);
        }
        
        // 制御失敗検出
        static float last_error = 0;
        float position_error = state.position.magnitude();
        if (position_error > 10.0) {  // 10m以上の偏差
            triggerEmergency(CONTROL_FAILURE);
        }
        
        // センサ異常検出
        if (isnan(state.attitude.w) || 
            state.imu.acceleration.magnitude() > 50) {
            triggerEmergency(SENSOR_FAILURE);
        }
    }
    
    void triggerEmergency(EmergencyType type) {
        if (emergency_active) return;  // 既に緊急状態
        
        emergency_active = true;
        
        ESP_LOGE("EMERGENCY", "%s", getEmergencyString(type).c_str());
        
        // 該当する緊急手順を実行
        for (auto& action : emergency_actions) {
            if (action.type == type && !action.executed) {
                action.action_function();
                action.executed = true;
                break;
            }
        }
        
        // 緊急信号の送信
        sendEmergencySignal(type);
    }
    
private:
    static void handleCommLoss() {
        ESP_LOGW("EMERGENCY", "Executing communication loss procedure");
        // 1. 現在位置でホバリング
        flight_controller.setMode(POSITION_HOLD);
        delay(5000);
        
        // 2. ホームポジションへ帰還
        flight_controller.setMode(RETURN_TO_LAUNCH);
        
        // 3. 30秒後に自動着陸
        delay(30000);
        flight_controller.setMode(AUTO_LAND);
    }
    
    static void handleLowBattery() {
        ESP_LOGW("EMERGENCY", "Executing low battery procedure");
        // 即座に着陸モードに移行
        flight_controller.setMode(AUTO_LAND);
        
        // 高降下率で素早く着陸
        flight_controller.setDescentRate(2.0);  // m/s
    }
    
    static void handleControlFailure() {
        ESP_LOGE("EMERGENCY", "Executing control failure procedure");
        // 全ての制御ループを無効化
        flight_controller.disableAllControllers();
        
        // 最小推力で緊急着陸
        setAllMotors(MIN_THROTTLE + 50);
        
        // 5秒後に完全停止
        delay(5000);
        setAllMotors(0);
    }
    
    static void handleSensorFailure() {
        ESP_LOGE("EMERGENCY", "Executing sensor failure procedure");
        // センサフュージョンを停止
        flight_controller.disableSensorFusion();
        
        // 最後の有効な姿勢を維持
        flight_controller.setMode(ATTITUDE_HOLD);
        
        // ゆっくり着陸
        flight_controller.setMode(AUTO_LAND);
    }
    
    static void handleExternalAbort() {
        ESP_LOGE("EMERGENCY", "Executing external abort procedure");
        // 即座にモータ停止
        setAllMotors(0);
        flight_controller.disarm();
    }
    
    String getEmergencyString(EmergencyType type) {
        switch (type) {
            case COMMUNICATION_LOSS: return "COMMUNICATION_LOSS";
            case LOW_BATTERY: return "LOW_BATTERY";
            case CONTROL_FAILURE: return "CONTROL_FAILURE";
            case SENSOR_FAILURE: return "SENSOR_FAILURE";
            case EXTERNAL_ABORT: return "EXTERNAL_ABORT";
            default: return "UNKNOWN";
        }
    }
    
    void sendEmergencySignal(EmergencyType type) {
        // 地上局への緊急信号送信
        EmergencyMessage msg;
        msg.type = type;
        msg.timestamp = millis();
        msg.position = flight_controller.getCurrentPosition();
        msg.battery_level = flight_controller.getBatteryVoltage();
        
        // WiFi/無線での送信
        transmitEmergencyMessage(msg);
        
        // LED/ブザーでの視覚・音響警告
        setLED(RED, BLINK_FAST);
        setBuzzer(1000, 100);  // 1kHz, 100ms
    }
};
```

## 6.4 性能評価と最適化

### 6.4.1 飛行性能メトリクス

```cpp
class PerformanceAnalyzer {
private:
    struct FlightData {
        uint32_t timestamp;
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        Quaternion attitude;
        Vector3 angular_velocity;
        MotorCommands motor_commands;
        float battery_voltage;
    };
    
    vector<FlightData> flight_log;
    
public:
    void logFlightData(const State& state, const MotorCommands& commands) {
        FlightData data;
        data.timestamp = millis();
        data.position = state.position;
        data.velocity = state.velocity;
        data.acceleration = state.acceleration;
        data.attitude = state.attitude;
        data.angular_velocity = state.angular_velocity;
        data.motor_commands = commands;
        data.battery_voltage = state.battery_voltage;
        
        flight_log.push_back(data);
        
        // メモリ制限のため古いデータを削除
        if (flight_log.size() > 10000) {
            flight_log.erase(flight_log.begin());
        }
    }
    
    struct PerformanceMetrics {
        // 制御性能
        float position_rmse;
        float attitude_rmse;
        float settling_time;
        float overshoot_percentage;
        
        // エネルギー効率
        float average_power;
        float energy_consumption;
        float flight_time_per_mah;
        
        // 安定性
        float vibration_level;
        float control_smoothness;
        float disturbance_rejection;
        
        // 応答性
        float response_time;
        float bandwidth;
        float phase_margin;
    };
    
    PerformanceMetrics analyzePerformance() {
        PerformanceMetrics metrics;
        
        if (flight_log.size() < 100) {
            ESP_LOGW("ANALYSIS", "Insufficient data for analysis");
            return metrics;
        }
        
        // 制御性能の解析
        metrics.position_rmse = calculatePositionRMSE();
        metrics.attitude_rmse = calculateAttitudeRMSE();
        metrics.settling_time = calculateSettlingTime();
        metrics.overshoot_percentage = calculateOvershoot();
        
        // エネルギー効率の解析
        metrics.average_power = calculateAveragePower();
        metrics.energy_consumption = calculateEnergyConsumption();
        metrics.flight_time_per_mah = calculateFlightTimePerMAh();
        
        // 安定性の解析
        metrics.vibration_level = calculateVibrationLevel();
        metrics.control_smoothness = calculateControlSmoothness();
        
        // 応答性の解析
        metrics.response_time = calculateResponseTime();
        
        // 結果の表示
        printPerformanceReport(metrics);
        
        return metrics;
    }
    
private:
    float calculatePositionRMSE() {
        if (flight_log.empty()) return 0;
        
        float sum_squared_error = 0;
        int count = 0;
        
        // 目標位置を推定（最頻値または設定値）
        Vector3 target_position = estimateTargetPosition();
        
        for (const auto& data : flight_log) {
            Vector3 error = data.position - target_position;
            sum_squared_error += error.magnitudeSquared();
            count++;
        }
        
        return sqrt(sum_squared_error / count);
    }
    
    float calculateVibrationLevel() {
        if (flight_log.size() < 3) return 0;
        
        float vibration_sum = 0;
        int count = 0;
        
        for (int i = 2; i < flight_log.size(); i++) {
            // 2次微分（加速度の変化）を計算
            Vector3 jerk = (flight_log[i].acceleration - 
                           2 * flight_log[i-1].acceleration + 
                           flight_log[i-2].acceleration);
            
            vibration_sum += jerk.magnitude();
            count++;
        }
        
        return vibration_sum / count;
    }
    
    float calculateControlSmoothness() {
        if (flight_log.size() < 2) return 0;
        
        float smoothness_sum = 0;
        int count = 0;
        
        for (int i = 1; i < flight_log.size(); i++) {
            // モータ指令の変化率
            float motor_change = 0;
            for (int j = 0; j < 4; j++) {
                float change = abs(flight_log[i].motor_commands.motors[j] - 
                                 flight_log[i-1].motor_commands.motors[j]);
                motor_change += change;
            }
            
            smoothness_sum += motor_change;
            count++;
        }
        
        return smoothness_sum / count;
    }
    
    float calculateAveragePower() {
        if (flight_log.empty()) return 0;
        
        float power_sum = 0;
        int count = 0;
        
        for (const auto& data : flight_log) {
            // 推定電力計算（電圧×電流）
            float total_throttle = 0;
            for (int i = 0; i < 4; i++) {
                total_throttle += data.motor_commands.motors[i];
            }
            
            // 簡略化された電力モデル
            float estimated_current = total_throttle / 4000.0 * 20.0;  // 最大20A
            float power = data.battery_voltage * estimated_current;
            
            power_sum += power;
            count++;
        }
        
        return power_sum / count;
    }
    
    Vector3 estimateTargetPosition() {
        // 位置データの最頻値を計算（ホバリング位置の推定）
        Vector3 sum(0, 0, 0);
        int count = 0;
        
        // 安定した期間のデータのみ使用
        for (int i = flight_log.size() / 4; i < 3 * flight_log.size() / 4; i++) {
            sum += flight_log[i].position;
            count++;
        }
        
        return sum / count;
    }
    
    void printPerformanceReport(const PerformanceMetrics& metrics) {
        ESP_LOGI("ANALYSIS", "=== Performance Analysis Report ===");
        ESP_LOGI("ANALYSIS", "Position RMSE: %.3f m", metrics.position_rmse);
        ESP_LOGI("ANALYSIS", "Attitude RMSE: %.3f rad", metrics.attitude_rmse);
        ESP_LOGI("ANALYSIS", "Average Power: %.3f W", metrics.average_power);
        ESP_LOGI("ANALYSIS", "Vibration Level: %.3f m/s³", metrics.vibration_level);
        ESP_LOGI("ANALYSIS", "Control Smoothness: %.3f", metrics.control_smoothness);
        ESP_LOGI("ANALYSIS", "===================================");
    }
};
```

### 6.4.2 リアルタイム監視システム

```cpp
class RealTimeMonitor {
private:
    struct MonitorData {
        float cpu_usage;
        float memory_usage;
        uint32_t loop_frequency;
        uint32_t max_loop_time;
        uint32_t communication_latency;
        uint8_t packet_loss_rate;
    } system_stats;
    
    uint32_t last_loop_time = 0;
    uint32_t loop_count = 0;
    uint32_t max_execution_time = 0;
    
public:
    void updateSystemStats() {
        // CPU使用率の計算
        uint32_t now = micros();
        uint32_t loop_time = now - last_loop_time;
        
        if (loop_time > max_execution_time) {
            max_execution_time = loop_time;
        }
        
        system_stats.max_loop_time = max_execution_time;
        
        // ループ周波数の計算
        loop_count++;
        static uint32_t freq_calc_start = millis();
        if (millis() - freq_calc_start >= 1000) {  // 1秒毎
            system_stats.loop_frequency = loop_count;
            loop_count = 0;
            freq_calc_start = millis();
            max_execution_time = 0;  // リセット
        }
        
        // メモリ使用率（ESP32の場合）
        system_stats.memory_usage = 100.0 * (1.0 - (float)ESP.getFreeHeap() / ESP.getHeapSize());
        
        last_loop_time = now;
    }
    
    void checkPerformanceWarnings() {
        // 制御周波数の監視
        if (system_stats.loop_frequency < 450) {  // 500Hz目標に対して
            ESP_LOGW("PERFORMANCE", "WARNING: Control frequency too low");
        }
        
        // 実行時間の監視
        if (system_stats.max_loop_time > 2500) {  // 2.5ms以上
            ESP_LOGW("PERFORMANCE", "WARNING: Control loop execution time too high");
        }
        
        // メモリ使用率の監視
        if (system_stats.memory_usage > 80) {
            ESP_LOGW("PERFORMANCE", "WARNING: High memory usage");
        }
    }
    
    void sendTelemetry() {
        // 地上局への監視データ送信
        TelemetryPacket packet;
        packet.cpu_usage = system_stats.cpu_usage;
        packet.memory_usage = system_stats.memory_usage;
        packet.loop_frequency = system_stats.loop_frequency;
        packet.max_loop_time = system_stats.max_loop_time;
        
        // WiFi経由で送信
        sendPacket(packet);
    }
};
```

### 6.4.3 自動レポート生成

```cpp
class TestReportGenerator {
private:
    struct TestSession {
        uint32_t start_time;
        uint32_t end_time;
        String test_name;
        String pilot_name;
        String weather_conditions;
        vector<String> notes;
        PerformanceMetrics metrics;
        bool success;
    };
    
    TestSession current_session;
    
public:
    void startTestSession(const String& test_name, const String& pilot) {
        current_session.start_time = millis();
        current_session.test_name = test_name;
        current_session.pilot_name = pilot;
        current_session.notes.clear();
        current_session.success = false;
        
        ESP_LOGI("LOGGER", "Test session started: %s", test_name.c_str());
    }
    
    void endTestSession(bool success, const PerformanceMetrics& metrics) {
        current_session.end_time = millis();
        current_session.success = success;
        current_session.metrics = metrics;
        
        generateReport();
    }
    
    void addNote(const String& note) {
        current_session.notes.push_back(note);
    }
    
private:
    void generateReport() {
        String report = generateHTMLReport();
        saveReport(report);
        
        ESP_LOGI("LOGGER", "Test report generated");
    }
    
    String generateHTMLReport() {
        String html = "<!DOCTYPE html><html><head><title>Flight Test Report</title></head><body>";
        
        // ヘッダー情報
        html += "<h1>Flight Test Report</h1>";
        html += "<table border='1'>";
        html += "<tr><td>Test Name</td><td>" + current_session.test_name + "</td></tr>";
        html += "<tr><td>Pilot</td><td>" + current_session.pilot_name + "</td></tr>";
        html += "<tr><td>Start Time</td><td>" + String(current_session.start_time) + "</td></tr>";
        html += "<tr><td>Duration</td><td>" + String((current_session.end_time - current_session.start_time) / 1000) + " seconds</td></tr>";
        html += "<tr><td>Result</td><td>" + String(current_session.success ? "SUCCESS" : "FAILURE") + "</td></tr>";
        html += "</table>";
        
        // 性能メトリクス
        html += "<h2>Performance Metrics</h2>";
        html += "<table border='1'>";
        html += "<tr><td>Position RMSE</td><td>" + String(current_session.metrics.position_rmse) + " m</td></tr>";
        html += "<tr><td>Attitude RMSE</td><td>" + String(current_session.metrics.attitude_rmse) + " rad</td></tr>";
        html += "<tr><td>Average Power</td><td>" + String(current_session.metrics.average_power) + " W</td></tr>";
        html += "<tr><td>Vibration Level</td><td>" + String(current_session.metrics.vibration_level) + "</td></tr>";
        html += "</table>";
        
        // ノート
        html += "<h2>Test Notes</h2><ul>";
        for (const auto& note : current_session.notes) {
            html += "<li>" + note + "</li>";
        }
        html += "</ul>";
        
        html += "</body></html>";
        
        return html;
    }
    
    void saveReport(const String& report) {
        // SDカードまたはフラッシュメモリに保存
        String filename = "/test_report_" + String(current_session.start_time) + ".html";
        
        File file = SD.open(filename, FILE_WRITE);
        if (file) {
            file.print(report);
            file.close();
            ESP_LOGI("LOGGER", "Report saved: %s", filename.c_str());
        } else {
            ESP_LOGE("LOGGER", "Failed to save report");
        }
    }
};
```

## まとめ

本章では、M5StampFlyの制御システムをテストするための包括的な手法を解説しました：

1. **シミュレーション環境**：安全で効率的な開発とテスト
2. **制御パラメータ調整**：システム同定からPIDチューニングまで
3. **実機試験手順**：安全性を最優先とした段階的アプローチ
4. **性能評価**：客観的なメトリクスによる評価と最適化

重要なポイント：

- **安全第一**：全ての試験で安全を最優先
- **段階的アプローチ**：小さなステップで確実に進歩
- **データ駆動**：客観的なデータに基づく評価
- **継続的改善**：テスト結果を次の改良に活用

実機試験では予期しない問題が発生することが多いため、十分な準備と慎重な手順が重要です。シミュレーションでの事前検証と、実機での段階的テストを組み合わせることで、安全で効果的な開発が可能になります。

次章では、本シリーズの総まとめとして、今後の展望について述べます。

## 参考文献

### シミュレーション・テスト関連
- Poksawat, P., Wang, L., & Mohamed, A. (2017). Automatic Tuning of Attitude Control System for Fixed-Wing Aircraft. IEEE Access
- Rauw, M. O. (2001). FDC 1.2 - A Simulink Toolbox for Flight Dynamics and Control Analysis  
- Stevens, B. L., & Lewis, F. L. (2003). Aircraft Control and Simulation

### ドローンシミュレーション実践資料
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343) - 102.2K views
- [ドローンの2Dシミュレーション](https://www.docswell.com/s/Kouhei_Ito/K7D2NZ-2022-09-11-142552) - 4.3K views
- [プログラム可能なドローンを体験！](https://www.docswell.com/s/Kouhei_Ito/K7RYG1-2024-07-15-124836) - 10.8K views

---

*本記事は「StampFly制御システム完全ガイド」シリーズの第6章です。*