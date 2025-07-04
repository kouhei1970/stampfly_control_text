# 第4章 制御工学入門 - StampFly制御システム入門

## はじめに

前章では、マルチコプタの運動を数学的にモデル化する方法を学びました。本章では、そのモデルに基づいて制御システムを設計するための制御工学の基礎を学びます。特に、PID制御から始めて、より高度な制御手法へと段階的に理解を深めていきます。

## 4.1 制御系の基本概念

### 4.1.1 フィードバック制御

制御工学の中心となる概念は「フィードバック制御」です。

```
目標値(r) → [誤差] → [制御器] → [入力(u)] → [プラント] → 出力(y)
              ↑                                        ↓
              ←────────────── [センサ] ←─────────────────
```

#### 基本要素

1. **目標値（Reference）**：望ましい状態
2. **誤差（Error）**：目標値と現在値の差
3. **制御器（Controller）**：誤差から制御入力を計算
4. **プラント（Plant）**：制御対象（マルチコプタ）
5. **センサ（Sensor）**：状態を測定

### 4.1.2 制御の目的

マルチコプタ制御における主な目的：

1. **安定化**：不安定な系を安定にする
2. **追従性**：目標値への素早い収束
3. **外乱抑制**：風などの影響を最小化
4. **ロバスト性**：モデル誤差に対する頑健性

## 4.2 PID制御

### 4.2.1 PID制御の原理

PID制御は最も広く使われる制御手法で、3つの要素から構成されます：

- **P（比例）**：現在の誤差に比例した制御
- **I（積分）**：誤差の累積に比例した制御
- **D（微分）**：誤差の変化率に比例した制御

#### 制御則

```
u(t) = Kp × e(t) + Ki × ∫e(τ)dτ + Kd × de(t)/dt
```

### 4.2.2 高度PID制御クラスの実装

```cpp
// 高度PID制御クラス
class AdvancedPIDController {
public:
    struct PIDParameters {
        float kp = 1.0f;              // 比例ゲイン
        float ki = 0.0f;              // 積分ゲイン
        float kd = 0.0f;              // 微分ゲイン
        float integral_limit = 10.0f;  // 積分項制限
        float output_limit = 100.0f;   // 出力制限
        float derivative_filter_alpha = 0.1f; // 微分フィルタ係数
        
        PIDParameters() = default;
        PIDParameters(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    };
    
    struct PIDState {
        float error = 0.0f;           // 現在の誤差
        float prev_error = 0.0f;      // 前回の誤差
        float integral = 0.0f;        // 積分項
        float derivative = 0.0f;      // 微分項
        float filtered_derivative = 0.0f; // フィルタ後微分
        float output = 0.0f;          // 制御出力
        
        PIDState() = default;
    };
    
    struct PIDDiagnostics {
        float p_term = 0.0f;          // P項寄与
        float i_term = 0.0f;          // I項寄与
        float d_term = 0.0f;          // D項寄与
        bool integral_saturated = false; // 積分飽和状態
        bool output_saturated = false;   // 出力飽和状態
        
        PIDDiagnostics() = default;
    };
    
private:
    PIDParameters params_;
    PIDState state_;
    PIDDiagnostics diagnostics_;
    bool enable_derivative_on_measurement_ = true; // 微分キック対策
    float prev_measurement_ = 0.0f;
    bool is_initialized_ = false;
    
public:
    // コンストラクタ
    explicit AdvancedPIDController(const PIDParameters& params = PIDParameters())
        : params_(params) {}
    
    // メイン更新関数
    float update(float setpoint, float measurement, float dt) {
        // 初期化チェック
        if (!is_initialized_) {
            initialize(setpoint, measurement);
            return 0.0f;
        }
        
        // 誤差計算
        state_.error = setpoint - measurement;
        
        // P項計算
        diagnostics_.p_term = params_.kp * state_.error;
        
        // I項計算（アンチワインドアップ付き）
        updateIntegralTerm(dt);
        
        // D項計算（微分キック対策付き）
        updateDerivativeTerm(measurement, dt);
        
        // 総合出力計算
        float raw_output = diagnostics_.p_term + diagnostics_.i_term + diagnostics_.d_term;
        
        // 出力制限と飽和検出
        state_.output = constrainOutput(raw_output);
        
        // 状態更新
        state_.prev_error = state_.error;
        prev_measurement_ = measurement;
        
        return state_.output;
    }
    
private:
    // 初期化
    void initialize(float setpoint, float measurement) {
        state_.error = setpoint - measurement;
        state_.prev_error = state_.error;
        prev_measurement_ = measurement;
        is_initialized_ = true;
    }
    
    // 積分項更新
    void updateIntegralTerm(float dt) {
        // 積分飽和状態での更新停止チェック
        if (!diagnostics_.integral_saturated) {
            state_.integral += state_.error * dt;
        }
        
        // 積分項制限
        float prev_integral = state_.integral;
        state_.integral = std::clamp(state_.integral, -params_.integral_limit, params_.integral_limit);
        
        // 飽和検出
        diagnostics_.integral_saturated = (state_.integral != prev_integral);
        
        diagnostics_.i_term = params_.ki * state_.integral;
    }
    
    // 微分項更新
    void updateDerivativeTerm(float measurement, float dt) {
        if (dt <= 0.0f) {
            diagnostics_.d_term = 0.0f;
            return;
        }
        
        if (enable_derivative_on_measurement_) {
            // 微分キック対策: 誤差ではなく測定値の微分を使用
            state_.derivative = -(measurement - prev_measurement_) / dt;
        } else {
            // 伝統的な誤差微分
            state_.derivative = (state_.error - state_.prev_error) / dt;
        }
        
        // 微分フィルタ適用（ノイズ対策）
        state_.filtered_derivative = params_.derivative_filter_alpha * state_.derivative +
                                   (1.0f - params_.derivative_filter_alpha) * state_.filtered_derivative;
        
        diagnostics_.d_term = params_.kd * state_.filtered_derivative;
    }
    
    // 出力制限
    float constrainOutput(float raw_output) {
        float limited_output = std::clamp(raw_output, -params_.output_limit, params_.output_limit);
        diagnostics_.output_saturated = (limited_output != raw_output);
        return limited_output;
    }
    
public:
    // リセット関数
    void reset() {
        state_ = PIDState();
        diagnostics_ = PIDDiagnostics();
        is_initialized_ = false;
    }
    
    // パラメータ設定
    void setParameters(const PIDParameters& params) {
        params_ = params;
    }
    
    void setGains(float kp, float ki, float kd) {
        params_.kp = kp;
        params_.ki = ki;
        params_.kd = kd;
    }
    
    void enableDerivativeOnMeasurement(bool enable) {
        enable_derivative_on_measurement_ = enable;
    }
    
    // 取得関数
    const PIDParameters& getParameters() const {
        return params_;
    }
    
    const PIDState& getState() const {
        return state_;
    }
    
    const PIDDiagnostics& getDiagnostics() const {
        return diagnostics_;
    }
    
    // チューニング支援関数
    float getProportionalContribution() const {
        return diagnostics_.p_term;
    }
    
    float getIntegralContribution() const {
        return diagnostics_.i_term;
    }
    
    float getDerivativeContribution() const {
        return diagnostics_.d_term;
    }
    
    // パフォーマンス統計
    void printPerformanceStats() const {
        printf("PID Performance:\n");
        printf("  P-term: %6.3f, I-term: %6.3f, D-term: %6.3f\n",
               diagnostics_.p_term, diagnostics_.i_term, diagnostics_.d_term);
        printf("  Error: %6.3f, Output: %6.3f\n", state_.error, state_.output);
        printf("  Integral Sat: %s, Output Sat: %s\n",
               diagnostics_.integral_saturated ? "YES" : "NO",
               diagnostics_.output_saturated ? "YES" : "NO");
    }
};
```

### 4.2.3 PID各項の特性解析クラス

各PID項の効果を個別に解析するクラス：

```cpp
// PID各項の特性解析クラス
class PIDComponentAnalyzer {
public:
    struct ComponentResponse {
        float magnitude = 0.0f;       // 応答の大きさ
        float phase_lag = 0.0f;       // 位相遅れ [rad]
        float settling_time = 0.0f;   // 整定時間 [s]
        float overshoot = 0.0f;       // オーバーシュート [%]
        
        ComponentResponse() = default;
    };
    
private:
    AdvancedPIDController p_only_, i_only_, d_only_;
    
public:
    // コンストラクタ
    PIDComponentAnalyzer() {
        // 各項のみのPIDコントローラを初期化
        p_only_.setGains(1.0f, 0.0f, 0.0f);  // Pのみ
        i_only_.setGains(0.0f, 1.0f, 0.0f);  // Iのみ
        d_only_.setGains(0.0f, 0.0f, 1.0f);  // Dのみ
    }
    
    // P項特性解析
    ComponentResponse analyzeProportionalResponse(float kp, float step_input, 
                                                 float simulation_time = 5.0f, 
                                                 float dt = 0.01f) {
        ComponentResponse response;
        p_only_.setGains(kp, 0.0f, 0.0f);
        p_only_.reset();
        
        std::vector<float> output_history;
        float measurement = 0.0f;
        
        // シミュレーション実行
        for (float t = 0; t < simulation_time; t += dt) {
            float control_output = p_only_.update(step_input, measurement, dt);
            
            // 簡単な1次遅れシステムでシミュレーション
            float tau = 1.0f;  // 時定数
            measurement += (control_output - measurement) * dt / tau;
            
            output_history.push_back(measurement);
        }
        
        // 応答特性を解析
        response = analyzeStepResponse(output_history, step_input, dt);
        return response;
    }
    
    // I項特性解析
    ComponentResponse analyzeIntegralResponse(float ki, float step_input,
                                            float simulation_time = 10.0f,
                                            float dt = 0.01f) {
        ComponentResponse response;
        i_only_.setGains(0.0f, ki, 0.0f);
        i_only_.reset();
        
        std::vector<float> output_history;
        float measurement = 0.0f;
        
        for (float t = 0; t < simulation_time; t += dt) {
            float control_output = i_only_.update(step_input, measurement, dt);
            
            float tau = 1.0f;
            measurement += (control_output - measurement) * dt / tau;
            
            output_history.push_back(measurement);
        }
        
        response = analyzeStepResponse(output_history, step_input, dt);
        return response;
    }
    
    // D項特性解析
    ComponentResponse analyzeDerivativeResponse(float kd, float step_input,
                                              float simulation_time = 3.0f,
                                              float dt = 0.01f) {
        ComponentResponse response;
        d_only_.setGains(0.0f, 0.0f, kd);
        d_only_.reset();
        
        std::vector<float> output_history;
        float measurement = 0.0f;
        
        for (float t = 0; t < simulation_time; t += dt) {
            float control_output = d_only_.update(step_input, measurement, dt);
            
            float tau = 1.0f;
            measurement += (control_output - measurement) * dt / tau;
            
            output_history.push_back(measurement);
        }
        
        response = analyzeStepResponse(output_history, step_input, dt);
        return response;
    }
    
private:
    // ステップ応答解析
    ComponentResponse analyzeStepResponse(const std::vector<float>& response, 
                                        float target, float dt) {
        ComponentResponse result;
        
        if (response.empty()) return result;
        
        // 最終値と最大値を検出
        float final_value = response.back();
        float max_value = *std::max_element(response.begin(), response.end());
        
        // オーバーシュート計算
        if (final_value > 0.001f) {
            result.overshoot = std::max(0.0f, (max_value - final_value) / final_value * 100.0f);
        }
        
        // 整定時間計算（最終値の2%以内）
        float tolerance = 0.02f * std::abs(final_value);
        for (int i = response.size() - 1; i >= 0; --i) {
            if (std::abs(response[i] - final_value) > tolerance) {
                result.settling_time = (i + 1) * dt;
                break;
            }
        }
        
        result.magnitude = final_value;
        return result;
    }
    
public:
    // 統合解析レポート
    void generateComponentReport(float kp, float ki, float kd) {
        auto p_response = analyzeProportionalResponse(kp, 1.0f);
        auto i_response = analyzeIntegralResponse(ki, 1.0f);
        auto d_response = analyzeDerivativeResponse(kd, 1.0f);
        
        printf("PID Component Analysis Report:\n");
        printf("================================\n");
        
        printf("P-term (Kp=%.3f):\n", kp);
        printf("  Magnitude: %.3f\n", p_response.magnitude);
        printf("  Overshoot: %.1f%%\n", p_response.overshoot);
        printf("  Settling: %.3fs\n", p_response.settling_time);
        printf("  特徴: 速い応答、定常偏差あり\n\n");
        
        printf("I-term (Ki=%.3f):\n", ki);
        printf("  Magnitude: %.3f\n", i_response.magnitude);
        printf("  Settling: %.3fs\n", i_response.settling_time);
        printf("  特徴: 定常偏差除去、応答遅い\n\n");
        
        printf("D-term (Kd=%.3f):\n", kd);
        printf("  Initial kick: %.3f\n", d_response.magnitude);
        printf("  特徴: 予測制御、振動抑制\n\n");
    }
};
```

**特性：**
- 定常偏差を除去
- 応答が遅い
- オーバーシュートの原因

#### D項（微分制御）

```cpp
// 微分制御の効果
float derivativeControl(float error, float prev_error, float dt) {
    float derivative = (error - prev_error) / dt;
    return kd * derivative;
}
```

**特性：**
- 予測的な制御
- 振動を抑制
- ノイズに敏感

### 4.2.4 改良型PID制御

#### 微分キック対策

```cpp
// 測定値微分によるPID（微分キック防止）
class ImprovedPIDController {
private:
    float prev_measurement;
    
public:
    float update(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        // P項
        float p_term = kp * error;
        
        // I項
        integral += error * dt;
        float i_term = ki * integral;
        
        // D項（測定値の微分を使用）
        float d_measurement = (measurement - prev_measurement) / dt;
        float d_term = -kd * d_measurement;  // マイナス符号に注意
        
        prev_measurement = measurement;
        
        return p_term + i_term + d_term;
    }
};
```

#### ローパスフィルタ付き微分

```cpp
// 微分項にローパスフィルタを適用
class FilteredDerivative {
private:
    float alpha = 0.1;  // フィルタ係数
    float filtered_derivative = 0;
    
public:
    float update(float derivative) {
        // 一次ローパスフィルタ
        filtered_derivative = alpha * derivative + 
                             (1 - alpha) * filtered_derivative;
        return filtered_derivative;
    }
};
```

## 4.3 カスケード制御

### 4.3.1 多重ループ構造

マルチコプタでは、カスケード（多重ループ）制御がよく使われます：

```
位置目標 → [位置制御] → 速度目標 → [速度制御] → 姿勢目標 → [姿勢制御] → モータ指令
```

### 4.3.2 多層カスケード制御クラスの実装

```cpp
// 多層カスケード制御システム
class MultilayerCascadeController {
public:
    struct CascadeSetpoints {
        // 位置目標
        float x = 0.0f, y = 0.0f, z = 0.0f;          // [m]
        float yaw = 0.0f;                             // [rad]
        
        // 速度目標（外側ループから生成される）
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;       // [m/s]
        
        // 姿勢目標（中間ループから生成される）
        float roll = 0.0f, pitch = 0.0f;             // [rad]
        
        // 角速度目標（内側ループから生成される）
        float roll_rate = 0.0f, pitch_rate = 0.0f, yaw_rate = 0.0f; // [rad/s]
        
        CascadeSetpoints() = default;
    };
    
    struct MulticopterState {
        // 位置・速度
        float x = 0.0f, y = 0.0f, z = 0.0f;          // [m]
        float vx = 0.0f, vy = 0.0f, vz = 0.0f;       // [m/s]
        
        // 姿勢・角速度
        float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // [rad]
        float p = 0.0f, q = 0.0f, r = 0.0f;          // [rad/s]
        
        MulticopterState() = default;
    };
    
    struct MotorCommands {
        float motor1 = 0.0f, motor2 = 0.0f, motor3 = 0.0f, motor4 = 0.0f;
        
        MotorCommands() = default;
        MotorCommands(float m1, float m2, float m3, float m4) 
            : motor1(m1), motor2(m2), motor3(m3), motor4(m4) {}
        
        // 制限適用
        void clamp(float min_val = 0.0f, float max_val = 1.0f) {
            motor1 = std::clamp(motor1, min_val, max_val);
            motor2 = std::clamp(motor2, min_val, max_val);
            motor3 = std::clamp(motor3, min_val, max_val);
            motor4 = std::clamp(motor4, min_val, max_val);
        }
    };
    
    struct CascadeTimings {
        float attitude_freq = 500.0f;   // Hz（内側ループ）
        float velocity_freq = 100.0f;   // Hz（中間ループ）
        float position_freq = 50.0f;    // Hz（外側ループ）
        
        CascadeTimings() = default;
    };
    
private:
    // 各ループのPID制御器
    AdvancedPIDController position_x_controller_, position_y_controller_, position_z_controller_;
    AdvancedPIDController velocity_x_controller_, velocity_y_controller_, velocity_z_controller_;
    AdvancedPIDController attitude_roll_controller_, attitude_pitch_controller_, attitude_yaw_controller_;
    AdvancedPIDController rate_roll_controller_, rate_pitch_controller_, rate_yaw_controller_;
    
    CascadeSetpoints internal_setpoints_;
    CascadeTimings timings_;
    
    // タイミング管理
    float last_position_update_ = 0.0f;
    float last_velocity_update_ = 0.0f;
    float last_attitude_update_ = 0.0f;
    
    bool enable_position_control_ = true;
    bool enable_velocity_control_ = true;
    bool enable_attitude_control_ = true;
    
public:
    // コンストラクタ
    MultilayerCascadeController() {
        initializeControllers();
    }
    
    // 制御器の初期化
    void initializeControllers() {
        // 位置制御器（最も外側）
        position_x_controller_.setGains(1.0f, 0.0f, 0.0f);
        position_y_controller_.setGains(1.0f, 0.0f, 0.0f);
        position_z_controller_.setGains(2.0f, 0.5f, 0.1f);
        
        // 速度制御器（中間）
        velocity_x_controller_.setGains(2.0f, 0.1f, 0.0f);
        velocity_y_controller_.setGains(2.0f, 0.1f, 0.0f);
        velocity_z_controller_.setGains(5.0f, 1.0f, 0.2f);
        
        // 姿勢制御器
        attitude_roll_controller_.setGains(6.0f, 0.0f, 0.0f);
        attitude_pitch_controller_.setGains(6.0f, 0.0f, 0.0f);
        attitude_yaw_controller_.setGains(4.0f, 0.0f, 0.0f);
        
        // 角速度制御器（最も内側）
        rate_roll_controller_.setGains(0.1f, 0.02f, 0.005f);
        rate_pitch_controller_.setGains(0.1f, 0.02f, 0.005f);
        rate_yaw_controller_.setGains(0.08f, 0.01f, 0.0f);
    }
    
    // メイン制御ループ
    MotorCommands control(const MulticopterState& state, 
                         const CascadeSetpoints& setpoints, 
                         float current_time, float dt) {
        CascadeSetpoints working_setpoints = setpoints;
        
        // 位置制御ループ（最も低頻度）
        if (enable_position_control_ && 
            (current_time - last_position_update_) >= (1.0f / timings_.position_freq)) {
            updatePositionLoop(state, working_setpoints, dt);
            last_position_update_ = current_time;
        }
        
        // 速度制御ループ（中間頻度）
        if (enable_velocity_control_ && 
            (current_time - last_velocity_update_) >= (1.0f / timings_.velocity_freq)) {
            updateVelocityLoop(state, working_setpoints, dt);
            last_velocity_update_ = current_time;
        }
        
        // 姿勢制御ループ（最高頻度）
        if (enable_attitude_control_ && 
            (current_time - last_attitude_update_) >= (1.0f / timings_.attitude_freq)) {
            updateAttitudeLoop(state, working_setpoints, dt);
            last_attitude_update_ = current_time;
        }
        
        // モータミキシング
        return computeMotorMixing(internal_setpoints_, dt);
    }
    
private:
    // 位置制御ループ（外側）
    void updatePositionLoop(const MulticopterState& state, 
                           CascadeSetpoints& setpoints, float dt) {
        // X軸位置制御
        internal_setpoints_.vx = position_x_controller_.update(setpoints.x, state.x, dt);
        
        // Y軸位置制御
        internal_setpoints_.vy = position_y_controller_.update(setpoints.y, state.y, dt);
        
        // Z軸位置制御
        internal_setpoints_.vz = position_z_controller_.update(setpoints.z, state.z, dt);
        
        // 速度制限
        float max_velocity = 2.0f;  // m/s
        internal_setpoints_.vx = std::clamp(internal_setpoints_.vx, -max_velocity, max_velocity);
        internal_setpoints_.vy = std::clamp(internal_setpoints_.vy, -max_velocity, max_velocity);
        internal_setpoints_.vz = std::clamp(internal_setpoints_.vz, -1.0f, 1.0f);
    }
    
    // 速度制御ループ（中間）
    void updateVelocityLoop(const MulticopterState& state, 
                           CascadeSetpoints& setpoints, float dt) {
        // X軸速度制御 → ピッチ角目標
        internal_setpoints_.pitch = velocity_x_controller_.update(
            internal_setpoints_.vx, state.vx, dt);
        
        // Y軸速度制御 → ロール角目標（符号反転）
        internal_setpoints_.roll = -velocity_y_controller_.update(
            internal_setpoints_.vy, state.vy, dt);
        
        // Z軸速度制御 → スロットル
        float throttle_adjustment = velocity_z_controller_.update(
            internal_setpoints_.vz, state.vz, dt);
        
        // 姿勢角制限
        float max_angle = 30.0f * M_PI / 180.0f;  // 30度
        internal_setpoints_.roll = std::clamp(internal_setpoints_.roll, -max_angle, max_angle);
        internal_setpoints_.pitch = std::clamp(internal_setpoints_.pitch, -max_angle, max_angle);
    }
    
    // 姿勢制御ループ（内側）
    void updateAttitudeLoop(const MulticopterState& state, 
                           CascadeSetpoints& setpoints, float dt) {
        // ロール姿勢制御
        internal_setpoints_.roll_rate = attitude_roll_controller_.update(
            internal_setpoints_.roll, state.roll, dt);
        
        // ピッチ姿勢制御
        internal_setpoints_.pitch_rate = attitude_pitch_controller_.update(
            internal_setpoints_.pitch, state.pitch, dt);
        
        // ヨー姿勢制御
        internal_setpoints_.yaw_rate = attitude_yaw_controller_.update(
            setpoints.yaw, state.yaw, dt);
        
        // 角速度制限
        float max_rate = 180.0f * M_PI / 180.0f;  // 180 deg/s
        internal_setpoints_.roll_rate = std::clamp(internal_setpoints_.roll_rate, -max_rate, max_rate);
        internal_setpoints_.pitch_rate = std::clamp(internal_setpoints_.pitch_rate, -max_rate, max_rate);
        internal_setpoints_.yaw_rate = std::clamp(internal_setpoints_.yaw_rate, -max_rate, max_rate);
    }
    
    // モータミキシング
    MotorCommands computeMotorMixing(const CascadeSetpoints& setpoints, float dt) {
        // 角速度制御（最内ループ）
        float roll_output = rate_roll_controller_.update(setpoints.roll_rate, 
                                                        /* current_roll_rate */ 0.0f, dt);
        float pitch_output = rate_pitch_controller_.update(setpoints.pitch_rate, 
                                                          /* current_pitch_rate */ 0.0f, dt);
        float yaw_output = rate_yaw_controller_.update(setpoints.yaw_rate, 
                                                      /* current_yaw_rate */ 0.0f, dt);
        
        // 基本スロットル（重力補償）
        float base_throttle = 0.5f;  // ホバー時のスロットル
        
        // X配置でのモータミキシング
        MotorCommands commands;
        commands.motor1 = base_throttle + roll_output + pitch_output - yaw_output;  // 前右
        commands.motor2 = base_throttle - roll_output + pitch_output + yaw_output;  // 前左
        commands.motor3 = base_throttle - roll_output - pitch_output - yaw_output;  // 後左
        commands.motor4 = base_throttle + roll_output - pitch_output + yaw_output;  // 後右
        
        // 出力制限
        commands.clamp(0.0f, 1.0f);
        
        return commands;
    }
    
public:
    // 制御ループの有効/無効化
    void enablePositionControl(bool enable) { enable_position_control_ = enable; }
    void enableVelocityControl(bool enable) { enable_velocity_control_ = enable; }
    void enableAttitudeControl(bool enable) { enable_attitude_control_ = enable; }
    
    // タイミング設定
    void setCascadeTimings(const CascadeTimings& timings) { timings_ = timings; }
    
    // PIDゲイン調整
    void setPositionGains(float kp, float ki, float kd) {
        position_x_controller_.setGains(kp, ki, kd);
        position_y_controller_.setGains(kp, ki, kd);
    }
    
    void setVelocityGains(float kp, float ki, float kd) {
        velocity_x_controller_.setGains(kp, ki, kd);
        velocity_y_controller_.setGains(kp, ki, kd);
    }
    
    void setAttitudeGains(float kp, float ki, float kd) {
        attitude_roll_controller_.setGains(kp, ki, kd);
        attitude_pitch_controller_.setGains(kp, ki, kd);
    }
    
    void setRateGains(float kp, float ki, float kd) {
        rate_roll_controller_.setGains(kp, ki, kd);
        rate_pitch_controller_.setGains(kp, ki, kd);
        rate_yaw_controller_.setGains(kp * 0.8f, ki * 0.5f, kd * 0.0f); // ヨーは少し控えめ
    }
    
    // リセット
    void reset() {
        position_x_controller_.reset();
        position_y_controller_.reset();
        position_z_controller_.reset();
        velocity_x_controller_.reset();
        velocity_y_controller_.reset();
        velocity_z_controller_.reset();
        attitude_roll_controller_.reset();
        attitude_pitch_controller_.reset();
        attitude_yaw_controller_.reset();
        rate_roll_controller_.reset();
        rate_pitch_controller_.reset();
        rate_yaw_controller_.reset();
        
        internal_setpoints_ = CascadeSetpoints();
    }
};
```

### 4.3.3 カスケード設計支援クラス

```cpp
// カスケード制御の設計と調整を支援するクラス
class CascadeDesignHelper {
public:
    struct BandwidthSeparation {
        float position_bandwidth = 0.5f;    // Hz
        float velocity_bandwidth = 2.0f;    // Hz
        float attitude_bandwidth = 10.0f;   // Hz
        float rate_bandwidth = 50.0f;       // Hz
        
        BandwidthSeparation() = default;
    };
    
    struct StabilityMargins {
        float gain_margin_db = 6.0f;        // dB
        float phase_margin_deg = 45.0f;     // degrees
        float delay_margin_ms = 5.0f;       // ms
        
        StabilityMargins() = default;
    };
    
private:
    BandwidthSeparation bandwidths_;
    StabilityMargins margins_;
    
public:
    // コンストラクタ
    CascadeDesignHelper(const BandwidthSeparation& bw = BandwidthSeparation(),
                       const StabilityMargins& margins = StabilityMargins())
        : bandwidths_(bw), margins_(margins) {}
    
    // 帯域分離原則チェック
    bool checkBandwidthSeparation() const {
        // 各ループの帯域が適切に分離されているかチェック
        float min_separation_ratio = 3.0f;  // 最低3倍の分離
        
        return (bandwidths_.velocity_bandwidth >= 
                bandwidths_.position_bandwidth * min_separation_ratio) &&
               (bandwidths_.attitude_bandwidth >= 
                bandwidths_.velocity_bandwidth * min_separation_ratio) &&
               (bandwidths_.rate_bandwidth >= 
                bandwidths_.attitude_bandwidth * min_separation_ratio);
    }
    
    // 推奨PIDゲインの計算
    AdvancedPIDController::PIDParameters calculateRecommendedGains(
        float bandwidth_hz, float damping_ratio = 0.707f) const {
        
        AdvancedPIDController::PIDParameters gains;
        
        float omega_n = 2.0f * M_PI * bandwidth_hz;
        
        // 2次系の標準形から導出
        gains.kp = omega_n * omega_n;
        gains.ki = 0.0f;  // 通常は積分項は使わない（カスケードの外側で処理）
        gains.kd = 2.0f * damping_ratio * omega_n;
        
        return gains;
    }
    
    // 安定性解析（簡易版）
    bool analyzeStability(const MultilayerCascadeController& controller) const {
        // 実際の実装では、伝達関数解析やナイキスト判定等を行う
        // ここでは簡略化された判定
        
        bool bandwidth_ok = checkBandwidthSeparation();
        bool gains_reasonable = true;  // ゲインの妥当性チェック（省略）
        
        return bandwidth_ok && gains_reasonable;
    }
    
    // 調整ガイダンス出力
    void printTuningGuide() const {
        printf("=== カスケード制御調整ガイド ===\n");
        printf("1. 内側ループから調整開始\n");
        printf("   - Rate loop (角速度): 最初に調整\n");
        printf("   - Attitude loop (姿勢): 次に調整\n");
        printf("   - Velocity loop (速度): その次\n");
        printf("   - Position loop (位置): 最後\n\n");
        
        printf("2. 帯域分離の確認\n");
        printf("   - Position: %.1f Hz\n", bandwidths_.position_bandwidth);
        printf("   - Velocity: %.1f Hz\n", bandwidths_.velocity_bandwidth);
        printf("   - Attitude: %.1f Hz\n", bandwidths_.attitude_bandwidth);
        printf("   - Rate: %.1f Hz\n", bandwidths_.rate_bandwidth);
        printf("   分離OK: %s\n\n", checkBandwidthSeparation() ? "Yes" : "No");
        
        printf("3. 調整手順\n");
        printf("   - P項から調整開始\n");
        printf("   - 振動が出ない範囲でP項を上げる\n");
        printf("   - 必要に応じてD項で振動抑制\n");
        printf("   - I項は定常偏差がある場合のみ\n");
    }
};
```

## 4.4 状態空間表現と現代制御理論

### 4.4.1 状態空間モデル

システムを行列形式で表現：

```
ẋ = Ax + Bu
y = Cx + Du
```

ここで：
- **x**: 状態ベクトル
- **u**: 入力ベクトル
- **y**: 出力ベクトル
- **A**: システム行列
- **B**: 入力行列
- **C**: 出力行列
- **D**: 直達行列

### 4.4.2 線形化モデルの構築

ホバリング点周りでの線形化：

```cpp
// 簡略化された高度制御の状態空間モデル
class AltitudeStateSpace {
private:
    // 状態: x = [z, vz]^T
    // 入力: u = thrust
    float A[2][2] = {{0, 1}, {0, 0}};
    float B[2][1] = {{0}, {1/MASS}};
    float C[1][2] = {{1, 0}};  // 高度のみ観測
    
public:
    void updateState(float state[2], float input, float dt) {
        float x_dot[2];
        
        // ẋ = Ax + Bu
        x_dot[0] = A[0][0] * state[0] + A[0][1] * state[1] + B[0][0] * input;
        x_dot[1] = A[1][0] * state[0] + A[1][1] * state[1] + B[1][0] * input;
        
        // オイラー法による積分
        state[0] += x_dot[0] * dt;
        state[1] += x_dot[1] * dt;
    }
};
```

### 4.4.3 可制御性と可観測性

システムが制御可能かどうかを判定：

```cpp
// 可制御性行列の計算
void computeControllabilityMatrix(float A[][N], float B[][M], 
                                  float P[][N*M], int n, int m) {
    // P = [B, AB, A²B, ..., A^(n-1)B]
    
    // B をコピー
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            P[i][j] = B[i][j];
        }
    }
    
    // A^k B を計算
    float temp[N][M];
    memcpy(temp, B, sizeof(float) * n * m);
    
    for (int k = 1; k < n; k++) {
        matrixMultiply(A, temp, temp, n, n, m);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                P[i][k*m + j] = temp[i][j];
            }
        }
    }
}

// ランクの確認（簡略版）
bool isControllable(float P[][N*M], int n, int m) {
    // 実際にはSVD等でランクを計算
    return computeRank(P, n, n*m) == n;
}
```

## 4.5 LQR（線形二次レギュレータ）

### 4.5.1 最適制御問題

評価関数を最小化する制御則を求める：

```
J = ∫[x^T Q x + u^T R u] dt
```

ここで：
- **Q**: 状態の重み行列
- **R**: 入力の重み行列

### 4.5.2 LQRゲインの計算

リカッチ方程式を解いてゲインを求める：

```cpp
// 簡略化されたLQR実装（2次元の例）
class LQRController {
private:
    float K[1][2];  // フィードバックゲイン
    
public:
    LQRController(float q1, float q2, float r) {
        // Q = diag(q1, q2), R = r
        // 定常リカッチ方程式を解く（ここでは結果のみ）
        float a = sqrt(q1/r);
        float b = sqrt(q2/r + a*a);
        
        K[0][0] = a;
        K[0][1] = b;
    }
    
    float computeControl(float state[2]) {
        // u = -Kx
        return -(K[0][0] * state[0] + K[0][1] * state[1]);
    }
};
```

### 4.5.3 重み行列の選定

```cpp
// 重み行列の設計指針
struct LQRWeights {
    // 状態の重み（大きいほど該当状態を重視）
    float Q_position = 100;    // 位置誤差
    float Q_velocity = 10;     // 速度誤差
    float Q_attitude = 1000;   // 姿勢誤差（重要）
    float Q_rate = 1;         // 角速度誤差
    
    // 入力の重み（大きいほど入力を抑制）
    float R_thrust = 1;       // 推力
    float R_moment = 0.1;     // モーメント
};
```

## 4.6 安定性解析

### 4.6.1 リアプノフ安定性

システムの安定性を数学的に保証：

```cpp
// リアプノフ関数の候補
float lyapunovFunction(const State& state) {
    // V(x) = x^T P x （二次形式）
    float V = 0;
    
    // 簡単な例：エネルギーベースのリアプノフ関数
    V += 0.5 * MASS * (state.vx*state.vx + state.vy*state.vy + state.vz*state.vz);
    V += MASS * GRAVITY * state.z;
    V += 0.5 * (Ixx*state.p*state.p + Iyy*state.q*state.q + Izz*state.r*state.r);
    
    return V;
}

// リアプノフ関数の時間微分
float lyapunovDerivative(const State& state, const Control& control) {
    // V̇ = ∂V/∂x × ẋ
    // 安定性の条件：V̇ < 0
    
    // 実装は省略（実際には状態微分を計算）
    return derivative;
}
```

### 4.6.2 周波数応答解析

```cpp
// ボード線図のための周波数応答計算
struct FrequencyResponse {
    float magnitude;  // ゲイン [dB]
    float phase;      // 位相 [deg]
};

FrequencyResponse computeFrequencyResponse(float omega, 
                                          const TransferFunction& tf) {
    // G(jω) を計算
    complex<float> s(0, omega);
    complex<float> response = tf.evaluate(s);
    
    FrequencyResponse fr;
    fr.magnitude = 20 * log10(abs(response));
    fr.phase = arg(response) * 180 / M_PI;
    
    return fr;
}

// 安定余裕の計算
struct StabilityMargins {
    float gain_margin;   // ゲイン余裕 [dB]
    float phase_margin;  // 位相余裕 [deg]
    
    bool isStable() {
        return (gain_margin > 6) && (phase_margin > 30);
    }
};
```

## 4.7 ロバスト制御

### 4.7.1 モデル不確かさ

実際のシステムには必ずモデル誤差が存在：

```cpp
// パラメータ不確かさの考慮
struct UncertainParameters {
    float mass_nominal = 1.0;
    float mass_uncertainty = 0.1;  // ±10%
    
    float Ixx_nominal = 0.01;
    float Ixx_uncertainty = 0.002; // ±20%
    
    // 最悪ケースの解析
    float getWorstCaseMass() {
        return mass_nominal + mass_uncertainty;
    }
};
```

### 4.7.2 H∞制御の概念

```cpp
// 感度関数とロバスト性
class RobustController {
private:
    // 感度関数 S = (1 + GK)^(-1)
    // 相補感度関数 T = GK(1 + GK)^(-1)
    
public:
    bool checkRobustness(float frequency) {
        // |S(jω)| < 1/|Wₛ(jω)| （性能仕様）
        // |T(jω)| < 1/|Wₜ(jω)| （ロバスト性仕様）
        
        float S_magnitude = computeSensitivity(frequency);
        float T_magnitude = computeComplementarySensitivity(frequency);
        
        return (S_magnitude < 2) && (T_magnitude < 1.5);
    }
};
```

## 4.8 離散時間制御

### 4.8.1 離散化

デジタル実装のための離散化：

```cpp
// 連続時間から離散時間への変換
class DiscreteController {
private:
    float Ts;  // サンプリング時間
    
    // 離散時間PID
    float kp, ki, kd;
    float integral_k = 0;
    float error_k_1 = 0;
    
public:
    DiscreteController(float sample_time) : Ts(sample_time) {}
    
    float update(float error_k) {
        // 台形則による積分
        integral_k += Ts * (error_k + error_k_1) / 2;
        
        // 後退差分による微分
        float derivative_k = (error_k - error_k_1) / Ts;
        
        // 制御出力
        float u_k = kp * error_k + ki * integral_k + kd * derivative_k;
        
        error_k_1 = error_k;
        return u_k;
    }
};
```

### 4.8.2 エイリアシングとフィルタ

```cpp
// アンチエイリアシングフィルタ
class AntiAliasingFilter {
private:
    float cutoff_freq;
    float a, b;  // フィルタ係数
    float y_prev = 0;
    
public:
    AntiAliasingFilter(float fc, float fs) {
        cutoff_freq = fc;
        float omega_c = 2 * M_PI * fc / fs;
        
        // 一次バターワースフィルタ
        a = omega_c / (omega_c + 2);
        b = (omega_c - 2) / (omega_c + 2);
    }
    
    float filter(float x) {
        float y = a * (x + x_prev) - b * y_prev;
        x_prev = x;
        y_prev = y;
        return y;
    }
};
```

## 4.9 実装上の考慮事項

### 4.9.1 計算遅延の補償

```cpp
// スミス予測器
class SmithPredictor {
private:
    float delay_time;
    CircularBuffer delay_buffer;
    
public:
    float compensate(float control_input, float measurement) {
        // モデルベースの予測
        float predicted = modelPredict(control_input, delay_time);
        
        // 遅延した制御入力
        float delayed_input = delay_buffer.get(delay_time);
        
        // 補償された測定値
        return measurement + (predicted - modelOutput(delayed_input));
    }
};
```

### 4.9.2 実時間制約

```cpp
// 実時間スケジューリング
class ControlScheduler {
private:
    struct Task {
        void (*function)();
        float period;
        float next_run;
        int priority;
    };
    
    vector<Task> tasks;
    
public:
    void addTask(void (*func)(), float period, int priority) {
        tasks.push_back({func, period, 0, priority});
    }
    
    void run() {
        float current_time = getCurrentTime();
        
        // 優先度順にソート
        sort(tasks.begin(), tasks.end(), 
             [](const Task& a, const Task& b) {
                 return a.priority > b.priority;
             });
        
        for (auto& task : tasks) {
            if (current_time >= task.next_run) {
                task.function();
                task.next_run += task.period;
            }
        }
    }
};
```

## 4.10 制御性能の評価

### 4.10.1 時間応答特性

```cpp
struct TimeResponseMetrics {
    float rise_time;        // 立ち上がり時間
    float settling_time;    // 整定時間
    float overshoot;        // オーバーシュート
    float steady_state_error; // 定常偏差
    
    void evaluate(const vector<float>& response, float target) {
        // 立ち上がり時間（10%→90%）
        int idx_10 = findFirstIndex(response, 0.1 * target);
        int idx_90 = findFirstIndex(response, 0.9 * target);
        rise_time = (idx_90 - idx_10) * SAMPLE_TIME;
        
        // オーバーシュート
        float max_value = *max_element(response.begin(), response.end());
        overshoot = (max_value - target) / target * 100;
        
        // 整定時間（±2%以内）
        settling_time = findSettlingTime(response, target, 0.02);
        
        // 定常偏差
        float final_value = response.back();
        steady_state_error = abs(final_value - target);
    }
};
```

### 4.10.2 周波数応答特性

```cpp
// クローズドループ帯域幅の計算
float computeBandwidth(const FrequencyResponse* fr, int num_points) {
    float gain_3db = fr[0].magnitude - 3;  // -3dB点
    
    for (int i = 1; i < num_points; i++) {
        if (fr[i].magnitude < gain_3db) {
            return fr[i].frequency;
        }
    }
    return 0;  // 見つからない場合
}
```

## まとめ

本章では、マルチコプタ制御に必要な制御工学の基礎を学びました：

1. **PID制御**：最も基本的で実用的な制御手法
2. **カスケード制御**：多重ループによる階層的制御
3. **状態空間表現**：現代制御理論の基礎
4. **安定性解析**：制御系の安定性保証
5. **ロバスト制御**：不確かさへの対処

重要なポイント：

- **PID制御の調整**：各ゲインの役割を理解して調整
- **カスケード構造**：内側ループを高速に
- **安定余裕**：ゲイン余裕6dB以上、位相余裕30度以上
- **離散化**：デジタル実装時の注意点

次章では、これらの制御理論をマルチコプタに適用し、実際の制御システムを構築する方法を詳しく見ていきます。

## 参考文献

### 制御理論書籍
- Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). Feedback Control of Dynamic Systems
- Åström, K. J., & Murray, R. M. (2021). Feedback Systems: An Introduction for Scientists and Engineers
- Skogestad, S., & Postlethwaite, I. (2005). Multivariable Feedback Control: Analysis and Design

### 実践的制御理論資料
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343) - 102.2K views
- [カルマンフィルタ](https://www.docswell.com/s/Kouhei_Ito/Z31ELK-2022-10-09-145221) - 17.2K views
- [PID制御によるドローンの飛行高度制御の検討](https://www.docswell.com/s/Kouhei_Ito/KWQ9JK-2022-09-11-113853) - 5.8K views
- [最適レギュレータ](https://www.docswell.com/s/Kouhei_Ito/KNQR8P-2023-01-14-113021)
- [LTIシステムの解と遷移行列](https://www.docswell.com/s/Kouhei_Ito/KNP2EQ-2022-11-26-085152)

---

*本記事は「StampFly制御システム完全ガイド」シリーズの第4章です。*