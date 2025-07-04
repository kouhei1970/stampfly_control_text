# 第2章 航空工学の基本 - StampFly制御システム入門

## はじめに

前章では、ドローンの概要とM5StampFlyについて紹介しました。本章では、マルチコプタが飛行する物理的な原理を理解するため、航空工学の基本を学びます。プロペラがどのように推力を生成し、機体がどのように制御されるのか、その仕組みを詳しく見ていきましょう。

## 2.1 マルチコプタの飛行原理

### 2.1.1 推力の生成

マルチコプタは、プロペラ（ロータ）の回転によって下向きに空気を押し出し、その反作用として上向きの推力を得ます。これはニュートンの第3法則（作用・反作用の法則）に基づいています。

#### 推力の計算式

プロペラが生成する推力は以下の式で表されます：

```
T = Ct × ρ × n² × D⁴
```

ここで：
- **T**: 推力 [N]
- **Ct**: 推力係数（プロペラの形状に依存）
- **ρ**: 空気密度 [kg/m³]（標準大気圧で約1.225）
- **n**: プロペラ回転数 [回転/秒]
- **D**: プロペラ直径 [m]

#### 実装例（C++オブジェクト指向）

```cpp
// 推力計算クラス
class ThrustCalculator {
public:
    struct PropellerParameters {
        float thrust_coefficient = 0.1f;  // 推力係数 Ct [-]
        float diameter = 0.065f;          // プロペラ直径 [m]
        float air_density = 1.225f;       // 空気密度 [kg/m³]
        
        PropellerParameters() = default;
        PropellerParameters(float ct, float d, float rho) 
            : thrust_coefficient(ct), diameter(d), air_density(rho) {}
    };
    
private:
    PropellerParameters params_;
    
public:
    // コンストラクタ
    explicit ThrustCalculator(const PropellerParameters& params = PropellerParameters())
        : params_(params) {}
    
    // 推力計算メインメソッド
    float calculateThrust(float rpm) const {
        float n = rpm / 60.0f;  // RPMから回転/秒への変換
        float thrust = params_.thrust_coefficient * params_.air_density * 
                      n * n * std::pow(params_.diameter, 4);
        return thrust;
    }
    
    // カスタムパラメータでの計算
    float calculateThrust(float rpm, const PropellerParameters& custom_params) const {
        float n = rpm / 60.0f;
        float thrust = custom_params.thrust_coefficient * custom_params.air_density * 
                      n * n * std::pow(custom_params.diameter, 4);
        return thrust;
    }
    
    // RPMから推力を範囲制限付きで計算
    float calculateClampedThrust(float rpm, float max_thrust = 1.0f) const {
        float thrust = calculateThrust(rpm);
        return std::clamp(thrust, 0.0f, max_thrust);
    }
    
    // 逆算: 必要な推力からRPMを求める
    float calculateRequiredRPM(float desired_thrust) const {
        if (desired_thrust <= 0.0f) return 0.0f;
        
        float coefficient = params_.thrust_coefficient * params_.air_density * 
                           std::pow(params_.diameter, 4);
        
        if (coefficient <= 0.0f) return 0.0f;
        
        float n = std::sqrt(desired_thrust / coefficient);
        return n * 60.0f;  // 回転/秒からRPMへ
    }
    
    // パラメータ設定
    void setParameters(const PropellerParameters& params) {
        params_ = params;
    }
    
    // パラメータ取得
    const PropellerParameters& getParameters() const {
        return params_;
    }
};
```

### 2.1.2 トルクと反トルク

プロペラが回転すると、機体には反対方向のトルク（反トルク）が発生します。クアッドコプタでは、対角のプロペラを同じ方向に、隣接するプロペラを逆方向に回転させることで、この反トルクを相殺しています。

```
プロペラ配置（上から見た図）：
  CW   CCW
   1    2
    \ /
     X
    / \
   4    3
  CCW   CW

CW: 時計回り、CCW: 反時計回り
```

## 2.2 マルチコプタに作用する力とモーメント

### 2.2.1 力とモーメント計算クラス

マルチコプタに作用する力とモーメントを管理するクラス：

```cpp
// 力とモーメント計算クラス
class MulticopterForces {
public:
    struct ForceVector {
        float fx, fy, fz;  // 力の成分 [N]
        
        ForceVector(float x = 0.0f, float y = 0.0f, float z = 0.0f) 
            : fx(x), fy(y), fz(z) {}
        
        ForceVector operator+(const ForceVector& other) const {
            return ForceVector(fx + other.fx, fy + other.fy, fz + other.fz);
        }
        
        float magnitude() const {
            return std::sqrt(fx*fx + fy*fy + fz*fz);
        }
    };
    
    struct MomentVector {
        float mx, my, mz;  // モーメントの成分 [N・m]
        
        MomentVector(float x = 0.0f, float y = 0.0f, float z = 0.0f) 
            : mx(x), my(y), mz(z) {}
        
        MomentVector operator+(const MomentVector& other) const {
            return MomentVector(mx + other.mx, my + other.my, mz + other.mz);
        }
    };
    
    struct AerodynamicParameters {
        float mass = 0.0368f;            // 機体質量 [kg]
        float gravity = 9.81f;           // 重力加速度 [m/s²]
        float drag_coefficient = 0.1f;   // 抗力係数 [-]
        float frontal_area = 0.01f;      // 投影面積 [m²]
        float air_density = 1.225f;      // 空気密度 [kg/m³]
        
        AerodynamicParameters() = default;
    };
    
private:
    AerodynamicParameters params_;
    
public:
    // コンストラクタ
    explicit MulticopterForces(const AerodynamicParameters& params = AerodynamicParameters())
        : params_(params) {}
    
    // 重力計算（地球座標系）
    ForceVector calculateGravityForce() const {
        return ForceVector(0.0f, 0.0f, params_.mass * params_.gravity);
    }
    
    // 総推力計算
    ForceVector calculateTotalThrust(const std::array<float, 4>& motor_thrusts) const {
        float total_thrust = motor_thrusts[0] + motor_thrusts[1] + 
                           motor_thrusts[2] + motor_thrusts[3];
        return ForceVector(0.0f, 0.0f, -total_thrust);  // 上向きを負とするNED座標
    }
    
    // 空気抗力計算
    ForceVector calculateDragForce(const ForceVector& velocity) const {
        float v_magnitude = velocity.magnitude();
        if (v_magnitude < 1e-6f) {
            return ForceVector(0.0f, 0.0f, 0.0f);
        }
        
        float drag_magnitude = 0.5f * params_.drag_coefficient * params_.air_density * 
                              v_magnitude * v_magnitude * params_.frontal_area;
        
        // 速度と逆方向に作用
        float scale = -drag_magnitude / v_magnitude;
        return ForceVector(velocity.fx * scale, velocity.fy * scale, velocity.fz * scale);
    }
    
    // パラメータ設定
    void setParameters(const AerodynamicParameters& params) {
        params_ = params;
    }
    
    // パラメータ取得
    const AerodynamicParameters& getParameters() const {
        return params_;
    }
};
```

### 2.2.2 モーメント計算クラス

機体の回転を制御するモーメント計算クラス：

```cpp
// モーメント計算クラス
class MulticopterMoments {
public:
    struct GeometryParameters {
        float arm_length = 0.0325f;        // アーム長 [m] (81.5mm/2 * sqrt(2)/2)
        float moment_coefficient = 0.05f;   // モーメント係数 [-]
        
        GeometryParameters() = default;
        GeometryParameters(float arm, float moment_coeff) 
            : arm_length(arm), moment_coefficient(moment_coeff) {}
    };
    
private:
    GeometryParameters params_;
    
public:
    // コンストラクタ
    explicit MulticopterMoments(const GeometryParameters& params = GeometryParameters())
        : params_(params) {}
    
    // ロールモーメント計算（X軸周り）
    float calculateRollMoment(const std::array<float, 4>& motor_thrusts) const {
        // モータ配置: 0=左前, 1=右前, 2=右後, 3=左後
        return params_.arm_length * (-motor_thrusts[0] + motor_thrusts[1] + 
                                    motor_thrusts[2] - motor_thrusts[3]);
    }
    
    // ピッチモーメント計算（Y軸周り）
    float calculatePitchMoment(const std::array<float, 4>& motor_thrusts) const {
        return params_.arm_length * (motor_thrusts[0] + motor_thrusts[1] - 
                                    motor_thrusts[2] - motor_thrusts[3]);
    }
    
    // ヨーモーメント計算（Z軸周り）
    float calculateYawMoment(const std::array<float, 4>& motor_commands) const {
        // モータ回転方向: 0=CW, 1=CCW, 2=CW, 3=CCW
        return params_.moment_coefficient * (-motor_commands[0] + motor_commands[1] - 
                                           motor_commands[2] + motor_commands[3]);
    }
    
    // 全モーメント計算
    MulticopterForces::MomentVector calculateAllMoments(
        const std::array<float, 4>& motor_thrusts,
        const std::array<float, 4>& motor_commands) const {
        
        return MulticopterForces::MomentVector(
            calculateRollMoment(motor_thrusts),
            calculatePitchMoment(motor_thrusts),
            calculateYawMoment(motor_commands)
        );
    }
    
    // パラメータ設定
    void setParameters(const GeometryParameters& params) {
        params_ = params;
    }
    
    // パラメータ取得
    const GeometryParameters& getParameters() const {
        return params_;
    }
};
```

## 2.3 座標系の定義と変換

制御システムを設計する上で、座標系の理解は不可欠です。

### 2.3.1 機体座標系（Body Frame）

機体に固定された座標系：
- **X軸（Roll軸）**：機体前方向
- **Y軸（Pitch軸）**：機体右方向
- **Z軸（Yaw軸）**：機体下方向

### 2.3.2 地球座標系（Earth Frame / NED座標系）

地球に固定された座標系：
- **X軸**：北（North）
- **Y軸**：東（East）
- **Z軸**：下（Down）

### 2.3.3 座標変換クラス

機体座標系と地球座標系間の変換を管理するクラス：

```cpp
// 座標変換クラス
class CoordinateTransform {
public:
    struct EulerAngles {
        float roll, pitch, yaw;  // [rad]
        
        EulerAngles(float r = 0.0f, float p = 0.0f, float y = 0.0f)
            : roll(r), pitch(p), yaw(y) {}
    };
    
    struct Vector3 {
        float x, y, z;
        
        Vector3(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f)
            : x(x_), y(y_), z(z_) {}
        
        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }
        
        Vector3 operator*(float scale) const {
            return Vector3(x * scale, y * scale, z * scale);
        }
    };
    
    // 3x3回転行列クラス
    class RotationMatrix {
    private:
        float matrix_[3][3];
        
    public:
        RotationMatrix() {
            // 単位行列で初期化
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    matrix_[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }
        
        // オイラー角から回転行列を計算
        void setFromEuler(const EulerAngles& angles) {
            float cr = std::cos(angles.roll);   float sr = std::sin(angles.roll);
            float cp = std::cos(angles.pitch);  float sp = std::sin(angles.pitch);
            float cy = std::cos(angles.yaw);    float sy = std::sin(angles.yaw);
            
            matrix_[0][0] = cp * cy;
            matrix_[0][1] = sr * sp * cy - cr * sy;
            matrix_[0][2] = cr * sp * cy + sr * sy;
            
            matrix_[1][0] = cp * sy;
            matrix_[1][1] = sr * sp * sy + cr * cy;
            matrix_[1][2] = cr * sp * sy - sr * cy;
            
            matrix_[2][0] = -sp;
            matrix_[2][1] = sr * cp;
            matrix_[2][2] = cr * cp;
        }
        
        // ベクトルを回転（機体座標系から地球座標系へ）
        Vector3 transform(const Vector3& body_vector) const {
            return Vector3(
                matrix_[0][0] * body_vector.x + matrix_[0][1] * body_vector.y + matrix_[0][2] * body_vector.z,
                matrix_[1][0] * body_vector.x + matrix_[1][1] * body_vector.y + matrix_[1][2] * body_vector.z,
                matrix_[2][0] * body_vector.x + matrix_[2][1] * body_vector.y + matrix_[2][2] * body_vector.z
            );
        }
        
        // 逆変換（地球座標系から機体座標系へ）
        Vector3 inverseTransform(const Vector3& earth_vector) const {
            return Vector3(
                matrix_[0][0] * earth_vector.x + matrix_[1][0] * earth_vector.y + matrix_[2][0] * earth_vector.z,
                matrix_[0][1] * earth_vector.x + matrix_[1][1] * earth_vector.y + matrix_[2][1] * earth_vector.z,
                matrix_[0][2] * earth_vector.x + matrix_[1][2] * earth_vector.y + matrix_[2][2] * earth_vector.z
            );
        }
        
        // 行列要素のアクセス
        float operator()(int i, int j) const {
            return matrix_[i][j];
        }
        
        // 行列を直接取得
        void getMatrix(float R[3][3]) const {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    R[i][j] = matrix_[i][j];
                }
            }
        }
    };
    
private:
    RotationMatrix current_rotation_;
    
public:
    // コンストラクタ
    CoordinateTransform() = default;
    
    // 姿勢の更新
    void updateAttitude(const EulerAngles& angles) {
        current_rotation_.setFromEuler(angles);
    }
    
    // 機体座標系から地球座標系へ変換
    Vector3 bodyToEarth(const Vector3& body_vector) const {
        return current_rotation_.transform(body_vector);
    }
    
    // 地球座標系から機体座標系へ変換
    Vector3 earthToBody(const Vector3& earth_vector) const {
        return current_rotation_.inverseTransform(earth_vector);
    }
    
    // 現在の回転行列を取得
    const RotationMatrix& getRotationMatrix() const {
        return current_rotation_;
    }
    
    // 重力ベクトルを機体座標系で表現
    Vector3 getGravityInBodyFrame() const {
        Vector3 gravity_earth(0.0f, 0.0f, 9.81f);  // NED座標系での重力
        return earthToBody(gravity_earth);
    }
};
```

## 2.4 空力特性と安定性

### 2.4.1 静的安定性

マルチコプタは静的に不安定な系です。つまり、外乱を受けると元の状態に戻ろうとせず、むしろ発散してしまいます。

#### 不安定性の理由

1. **重心と推力中心の位置関係**
   - 推力の作用点が重心より上にある
   - わずかな傾きが増幅される

2. **空力的な復元力の欠如**
   - 固定翼機のような安定化効果がない

### 2.4.2 動的特性

マルチコプタの動的特性を理解することは、制御系設計に重要です：

#### 時定数

各運動モードの典型的な時定数：
- **姿勢変化**：0.1～0.5秒
- **位置変化**：1～5秒
- **高度変化**：2～10秒

## 2.5 マルチコプタの運動制御

### 2.5.1 モータ制御クラス

クアッドコプタの運動を制御するモータシステムクラス：

```cpp
// モータ制御クラス
class MotorController {
public:
    struct ControlInputs {
        float throttle = 0.0f;  // 上昇/下降 [-1.0 ~ 1.0]
        float roll = 0.0f;      // ロール [-1.0 ~ 1.0]
        float pitch = 0.0f;     // ピッチ [-1.0 ~ 1.0]
        float yaw = 0.0f;       // ヨー [-1.0 ~ 1.0]
        
        ControlInputs() = default;
        ControlInputs(float t, float r, float p, float y)
            : throttle(t), roll(r), pitch(p), yaw(y) {}
    };
    
    struct MotorOutputs {
        std::array<float, 4> motors;  // モータ出力 [0.0 ~ 1.0]
        
        MotorOutputs() {
            motors.fill(0.0f);
        }
        
        // インデックスアクセス
        float& operator[](size_t index) {
            return motors[index];
        }
        
        const float& operator[](size_t index) const {
            return motors[index];
        }
    };
    
    // モータインデックス定義
    enum MotorIndex {
        FRONT_LEFT = 0,   // 左前 (CW)
        FRONT_RIGHT = 1,  // 右前 (CCW)
        REAR_RIGHT = 2,   // 右後 (CW)
        REAR_LEFT = 3     // 左後 (CCW)
    };
    
private:
    float base_thrust_ = 0.5f;  // ベース推力 [0.0 ~ 1.0]
    float max_output_ = 1.0f;   // 最大出力
    float min_output_ = 0.0f;   // 最小出力
    
public:
    // コンストラクタ
    MotorController() = default;
    
    // メインミキシング関数
    MotorOutputs mixControls(const ControlInputs& inputs) const {
        MotorOutputs outputs;
        
        // X配置のクアッドコプタミキシングマトリックス
        outputs[FRONT_LEFT]  = base_thrust_ + inputs.throttle - inputs.roll + inputs.pitch - inputs.yaw;
        outputs[FRONT_RIGHT] = base_thrust_ + inputs.throttle + inputs.roll + inputs.pitch + inputs.yaw;
        outputs[REAR_RIGHT]  = base_thrust_ + inputs.throttle + inputs.roll - inputs.pitch - inputs.yaw;
        outputs[REAR_LEFT]   = base_thrust_ + inputs.throttle - inputs.roll - inputs.pitch + inputs.yaw;
        
        // 出力制限
        for (size_t i = 0; i < 4; ++i) {
            outputs[i] = std::clamp(outputs[i], min_output_, max_output_);
        }
        
        return outputs;
    }
    
    // 個別制御関数
    MotorOutputs throttleControl(float throttle_input) const {
        ControlInputs inputs(throttle_input, 0.0f, 0.0f, 0.0f);
        return mixControls(inputs);
    }
    
    MotorOutputs rollControl(float roll_input) const {
        ControlInputs inputs(0.0f, roll_input, 0.0f, 0.0f);
        return mixControls(inputs);
    }
    
    MotorOutputs pitchControl(float pitch_input) const {
        ControlInputs inputs(0.0f, 0.0f, pitch_input, 0.0f);
        return mixControls(inputs);
    }
    
    MotorOutputs yawControl(float yaw_input) const {
        ControlInputs inputs(0.0f, 0.0f, 0.0f, yaw_input);
        return mixControls(inputs);
    }
    
    // パラメータ設定
    void setBaseThrust(float base_thrust) {
        base_thrust_ = std::clamp(base_thrust, 0.0f, 1.0f);
    }
    
    void setOutputLimits(float min_output, float max_output) {
        min_output_ = min_output;
        max_output_ = max_output;
    }
    
    // パラメータ取得
    float getBaseThrust() const { return base_thrust_; }
    float getMaxOutput() const { return max_output_; }
    float getMinOutput() const { return min_output_; }
    
    // 出力の有効性チェック
    bool isOutputValid(const MotorOutputs& outputs) const {
        for (size_t i = 0; i < 4; ++i) {
            if (outputs[i] < min_output_ || outputs[i] > max_output_) {
                return false;
            }
        }
        return true;
    }
};
```

### 2.5.2 高度なモータミキシングクラス

実際の制御で使用する高度なモータミキシングクラス：

```cpp
// 高度モータミキシングクラス
class AdvancedMotorMixer : public MotorController {
public:
    struct MixingConfiguration {
        float throttle_scale = 1.0f;   // スロットルゲイン
        float roll_scale = 0.5f;       // ロールゲイン
        float pitch_scale = 0.5f;      // ピッチゲイン
        float yaw_scale = 0.3f;        // ヨーゲイン
        
        // モータ個別ゲイン（キャリブレーション用）
        std::array<float, 4> motor_gains = {1.0f, 1.0f, 1.0f, 1.0f};
        
        MixingConfiguration() = default;
    };
    
private:
    MixingConfiguration config_;
    bool anti_windup_enabled_ = true;
    
public:
    // コンストラクタ
    explicit AdvancedMotorMixer(const MixingConfiguration& config = MixingConfiguration())
        : config_(config) {}
    
    // スケール付きミキシング
    MotorOutputs mixControlsScaled(const ControlInputs& inputs) const {
        ControlInputs scaled_inputs;
        scaled_inputs.throttle = inputs.throttle * config_.throttle_scale;
        scaled_inputs.roll = inputs.roll * config_.roll_scale;
        scaled_inputs.pitch = inputs.pitch * config_.pitch_scale;
        scaled_inputs.yaw = inputs.yaw * config_.yaw_scale;
        
        MotorOutputs outputs = mixControls(scaled_inputs);
        
        // モータ個別ゲイン適用
        for (size_t i = 0; i < 4; ++i) {
            outputs[i] *= config_.motor_gains[i];
        }
        
        // アンチワインドアップ処理
        if (anti_windup_enabled_) {
            outputs = applyAntiWindup(outputs);
        }
        
        return outputs;
    }
    
    // アンチワインドアップ処理
    MotorOutputs applyAntiWindup(const MotorOutputs& raw_outputs) const {
        MotorOutputs outputs = raw_outputs;
        
        // 最大値を超えた場合のスケーリング
        float max_value = *std::max_element(outputs.motors.begin(), outputs.motors.end());
        if (max_value > getMaxOutput()) {
            float scale = getMaxOutput() / max_value;
            for (size_t i = 0; i < 4; ++i) {
                outputs[i] *= scale;
            }
        }
        
        // 最小値を下回った場合のオフセット
        float min_value = *std::min_element(outputs.motors.begin(), outputs.motors.end());
        if (min_value < getMinOutput()) {
            float offset = getMinOutput() - min_value;
            for (size_t i = 0; i < 4; ++i) {
                outputs[i] += offset;
                outputs[i] = std::clamp(outputs[i], getMinOutput(), getMaxOutput());
            }
        }
        
        return outputs;
    }
    
    // モータキャリブレーション
    void calibrateMotor(size_t motor_index, float gain) {
        if (motor_index < 4) {
            config_.motor_gains[motor_index] = std::clamp(gain, 0.5f, 2.0f);
        }
    }
    
    // 設定関数
    void setMixingConfiguration(const MixingConfiguration& config) {
        config_ = config;
    }
    
    void enableAntiWindup(bool enable) {
        anti_windup_enabled_ = enable;
    }
    
    // 取得関数
    const MixingConfiguration& getMixingConfiguration() const {
        return config_;
    }
    
    bool isAntiWindupEnabled() const {
        return anti_windup_enabled_;
    }
    
    // モータ出力の分析
    void analyzeOutputs(const MotorOutputs& outputs) const {
        float total = 0.0f;
        float max_val = outputs[0];
        float min_val = outputs[0];
        
        for (size_t i = 0; i < 4; ++i) {
            total += outputs[i];
            max_val = std::max(max_val, outputs[i]);
            min_val = std::min(min_val, outputs[i]);
        }
        
        printf("Motor Analysis: Avg=%.3f, Max=%.3f, Min=%.3f, Range=%.3f\n",
               total / 4.0f, max_val, min_val, max_val - min_val);
    }
};
```

## 2.6 プロペラの詳細設計

### 2.6.1 プロペラのパラメータ

プロペラの性能を決定する主要なパラメータ：

1. **直径（Diameter）**
   - 大きいほど効率的
   - 機体サイズの制約

2. **ピッチ（Pitch）**
   - プロペラが1回転で進む理論的な距離
   - 高ピッチ＝高速飛行向き
   - 低ピッチ＝ホバリング向き

3. **ブレード数**
   - 2枚：最も一般的、効率的
   - 3枚以上：推力は増えるが効率は低下

### 2.6.2 推力とトルクの関係

```cpp
// プロペラ性能解析クラス
class PropellerPerformanceAnalyzer {
public:
    struct PropellerSpecs {
        float diameter = 0.065f;        // 直径 [m]
        float pitch = 0.045f;           // ピッチ [m]
        int blade_count = 2;            // ブレード数
        float thrust_coefficient = 0.1f; // 推力係数 Ct
        float power_coefficient = 0.05f; // 動力係数 Cp
        
        PropellerSpecs() = default;
        PropellerSpecs(float d, float p, int blades = 2)
            : diameter(d), pitch(p), blade_count(blades) {}
    };
    
    struct PerformanceData {
        float thrust = 0.0f;     // 推力 [N]
        float torque = 0.0f;     // トルク [N・m]
        float power = 0.0f;      // 消費電力 [W]
        float efficiency = 0.0f; // 効率 [-]
        float figure_of_merit = 0.0f; // ホバリング効率
        
        PerformanceData() = default;
    };
    
private:
    PropellerSpecs specs_;
    float air_density_ = 1.225f;  // 空気密度 [kg/m³]
    
public:
    // コンストラクタ
    explicit PropellerPerformanceAnalyzer(const PropellerSpecs& specs = PropellerSpecs())
        : specs_(specs) {}
    
    // メイン性能計算
    PerformanceData calculatePerformance(float rpm, float forward_velocity = 0.0f) const {
        PerformanceData perf;
        
        if (rpm <= 0.0f) {
            return perf;  // ゼロ性能を返す
        }
        
        float n = rpm / 60.0f;  // 回転/秒へ変換
        
        // 基本性能計算
        perf.thrust = calculateThrust(n, forward_velocity);
        perf.power = calculatePower(n, forward_velocity);
        perf.torque = calculateTorque(perf.power, n);
        perf.efficiency = calculateEfficiency(perf.thrust, perf.power, forward_velocity);
        perf.figure_of_merit = calculateFigureOfMerit(perf.thrust, perf.power);
        
        return perf;
    }
    
    // 推力計算
    float calculateThrust(float n, float forward_velocity) const {
        // 進歩比（Advance Ratio）を考慮
        float advance_ratio = (n > 0.0f) ? forward_velocity / (n * specs_.diameter) : 0.0f;
        
        // 進歩比による推力係数の補正
        float ct_corrected = specs_.thrust_coefficient * (1.0f - 0.5f * advance_ratio);
        
        return ct_corrected * air_density_ * n * n * std::pow(specs_.diameter, 4);
    }
    
    // 動力計算
    float calculatePower(float n, float forward_velocity) const {
        float advance_ratio = (n > 0.0f) ? forward_velocity / (n * specs_.diameter) : 0.0f;
        
        // 進歩比による動力係数の補正
        float cp_corrected = specs_.power_coefficient * (1.0f + 0.2f * advance_ratio);
        
        return cp_corrected * air_density_ * n * n * n * std::pow(specs_.diameter, 5);
    }
    
    // トルク計算
    float calculateTorque(float power, float n) const {
        if (n <= 0.0f) return 0.0f;
        return power / (2.0f * M_PI * n);
    }
    
    // 効率計算
    float calculateEfficiency(float thrust, float power, float forward_velocity) const {
        if (power <= 0.0f) return 0.0f;
        
        // 有効出力（推進効率）
        float useful_power = thrust * forward_velocity;
        return useful_power / power;
    }
    
    // Figure of Merit（ホバリング効率）計算
    float calculateFigureOfMerit(float thrust, float power) const {
        if (power <= 0.0f || thrust <= 0.0f) return 0.0f;
        
        // 理想動力（運量理論）
        float disk_area = M_PI * specs_.diameter * specs_.diameter / 4.0f;
        float ideal_power = thrust * std::sqrt(thrust / (2.0f * air_density_ * disk_area));
        
        return ideal_power / power;
    }
    
    // パラメータ設定
    void setSpecs(const PropellerSpecs& specs) {
        specs_ = specs;
    }
    
    void setAirDensity(float air_density) {
        air_density_ = air_density;
    }
    
    // パラメータ取得
    const PropellerSpecs& getSpecs() const {
        return specs_;
    }
    
    float getAirDensity() const {
        return air_density_;
    }
};
```

## 2.7 環境要因の影響

### 2.7.1 環境要因管理クラス

飛行環境の変化を管理し、推力への影響を計算するクラス：

```cpp
// 環境要因管理クラス
class EnvironmentalFactors {
public:
    struct AtmosphericConditions {
        float altitude = 0.0f;        // 高度 [m]
        float temperature = 15.0f;    // 温度 [℃]
        float humidity = 50.0f;       // 相対湿度 [%]
        float pressure = 101325.0f;   // 気圧 [Pa]
        
        AtmosphericConditions() = default;
        AtmosphericConditions(float alt, float temp, float hum = 50.0f)
            : altitude(alt), temperature(temp), humidity(hum) {}
    };
    
    struct GroundEffectParameters {
        float rotor_diameter = 0.065f;   // ロータ直径 [m]
        float effect_threshold = 0.5f;   // 影響闾値（直径比）
        float max_effect = 0.15f;        // 最大影響率
        
        GroundEffectParameters() = default;
    };
    
private:
    AtmosphericConditions conditions_;
    GroundEffectParameters ground_params_;
    
    // 標準大気定数
    static constexpr float SEA_LEVEL_PRESSURE = 101325.0f;  // Pa
    static constexpr float SEA_LEVEL_TEMPERATURE = 288.15f; // K
    static constexpr float GAS_CONSTANT = 287.05f;          // J/(kg⋅K)
    static constexpr float LAPSE_RATE = 0.0065f;            // K/m
    
public:
    // コンストラクタ
    explicit EnvironmentalFactors(const AtmosphericConditions& conditions = AtmosphericConditions())
        : conditions_(conditions) {}
    
    // 空気密度計算（標準大気モデル）
    float calculateAirDensity() const {
        // 気圧の高度補正
        float pressure = SEA_LEVEL_PRESSURE * 
                        std::pow(1.0f - LAPSE_RATE * conditions_.altitude / SEA_LEVEL_TEMPERATURE, 5.256f);
        
        // 温度をKに変換
        float temperature_k = conditions_.temperature + 273.15f;
        
        // 湿度補正（簡略化）
        float humidity_factor = 1.0f - 0.0003f * conditions_.humidity;
        
        // 理想気体の状態方程式
        float density = (pressure / (GAS_CONSTANT * temperature_k)) * humidity_factor;
        
        return density;
    }
    
    // カスタム条件での空気密度計算
    float calculateAirDensity(float altitude, float temperature_celsius) const {
        AtmosphericConditions temp_conditions(altitude, temperature_celsius);
        EnvironmentalFactors temp_env(temp_conditions);
        return temp_env.calculateAirDensity();
    }
    
    // 地面効果係数計算
    float calculateGroundEffectFactor(float height_above_ground) const {
        // 高度が闾値を超えた場合は影響なし
        float threshold_height = ground_params_.rotor_diameter * ground_params_.effect_threshold;
        if (height_above_ground > threshold_height) {
            return 1.0f;
        }
        
        // 高度比で正規化
        float height_ratio = height_above_ground / ground_params_.rotor_diameter;
        
        // 指数的減衰モデル
        float effect_strength = ground_params_.max_effect * std::exp(-4.0f * height_ratio);
        
        return 1.0f + effect_strength;
    }
    
    // 総合環境補正係数
    float calculateTotalEnvironmentalFactor(float height_above_ground) const {
        float air_density_factor = calculateAirDensity() / 1.225f;  // 標準大気密度で正規化
        float ground_effect_factor = calculateGroundEffectFactor(height_above_ground);
        
        return air_density_factor * ground_effect_factor;
    }
    
    // 温度によるモータ性能補正
    float calculateMotorTemperatureFactor() const {
        // モータの温度特性（簡略化）
        float optimal_temp = 25.0f;  // ℃
        float temp_deviation = std::abs(conditions_.temperature - optimal_temp);
        
        // 温度偏差による性能低下
        float performance_factor = 1.0f - 0.001f * temp_deviation;
        return std::clamp(performance_factor, 0.8f, 1.1f);
    }
    
    // 設定関数
    void setConditions(const AtmosphericConditions& conditions) {
        conditions_ = conditions;
    }
    
    void setGroundParameters(const GroundEffectParameters& params) {
        ground_params_ = params;
    }
    
    void updateAltitude(float altitude) {
        conditions_.altitude = altitude;
    }
    
    void updateTemperature(float temperature) {
        conditions_.temperature = temperature;
    }
    
    // 取得関数
    const AtmosphericConditions& getConditions() const {
        return conditions_;
    }
    
    const GroundEffectParameters& getGroundParameters() const {
        return ground_params_;
    }
    
    // 環境状態の表示
    void printEnvironmentalStatus() const {
        printf("Environmental Status:\n");
        printf("  Altitude: %.1f m\n", conditions_.altitude);
        printf("  Temperature: %.1f C\n", conditions_.temperature);
        printf("  Air Density: %.3f kg/m3\n", calculateAirDensity());
        printf("  Density Factor: %.3f\n", calculateAirDensity() / 1.225f);
    }
};
```

## 2.8 実装における考慮事項

### 2.8.1 センサによる状態推定

実際の飛行では、以下のセンサを使用して機体の状態を推定します：

1. **IMU（慣性計測装置）**
   - 加速度センサ：3軸の加速度
   - ジャイロセンサ：3軸の角速度

2. **気圧センサ**
   - 高度推定

3. **磁気センサ**
   - 方位（ヨー角）の推定

### 2.8.2 高度姿勢推定クラス

センサフュージョンと高度な姿勢推定を実装するクラス：

```cpp
// 高度姿勢推定クラス
class AdvancedAttitudeEstimator {
public:
    struct IMUData {
        float accel_x, accel_y, accel_z;  // 加速度 [m/s²]
        float gyro_x, gyro_y, gyro_z;     // 角速度 [rad/s]
        
        IMUData() : accel_x(0), accel_y(0), accel_z(0),
                   gyro_x(0), gyro_y(0), gyro_z(0) {}
    };
    
    struct MagnetometerData {
        float mag_x, mag_y, mag_z;  // 磁界 [uT]
        
        MagnetometerData() : mag_x(0), mag_y(0), mag_z(0) {}
    };
    
    struct AttitudeData {
        float roll, pitch, yaw;     // オイラー角 [rad]
        float roll_rate, pitch_rate, yaw_rate;  // 角速度 [rad/s]
        
        AttitudeData() : roll(0), pitch(0), yaw(0),
                        roll_rate(0), pitch_rate(0), yaw_rate(0) {}
    };
    
    struct FilterParameters {
        float complementary_alpha = 0.98f;   // 相補フィルタ係数
        float gyro_drift_compensation = 0.001f; // ジャイロドリフト補正
        float accel_noise_threshold = 0.1f;   // 加速度ノイズ闾値
        float mag_declination = 0.0f;         // 磁気偏角 [rad]
        
        FilterParameters() = default;
    };
    
private:
    AttitudeData attitude_;
    FilterParameters params_;
    CoordinateTransform coordinate_transform_;
    
    // フィルタ状態
    float gyro_bias_x_ = 0.0f, gyro_bias_y_ = 0.0f, gyro_bias_z_ = 0.0f;
    bool is_initialized_ = false;
    float gravity_magnitude_ = 9.81f;
    
public:
    // コンストラクタ
    explicit AdvancedAttitudeEstimator(const FilterParameters& params = FilterParameters())
        : params_(params) {}
    
    // メイン更新関数
    void updateAttitude(const IMUData& imu, float dt, 
                       const MagnetometerData* mag = nullptr) {
        
        // 初期化チェック
        if (!is_initialized_) {
            initializeFromAccelerometer(imu);
            return;
        }
        
        // ジャイロバイアス補正
        float corrected_gx = imu.gyro_x - gyro_bias_x_;
        float corrected_gy = imu.gyro_y - gyro_bias_y_;
        float corrected_gz = imu.gyro_z - gyro_bias_z_;
        
        // ジャイロからの角度積分
        attitude_.roll += corrected_gx * dt;
        attitude_.pitch += corrected_gy * dt;
        attitude_.yaw += corrected_gz * dt;
        
        // 加速度センサからの姿勢推定
        AttitudeData accel_attitude = estimateFromAccelerometer(imu);
        
        // 相補フィルタ適用
        if (isAccelerometerReliable(imu)) {
            attitude_.roll = params_.complementary_alpha * attitude_.roll + 
                           (1.0f - params_.complementary_alpha) * accel_attitude.roll;
            attitude_.pitch = params_.complementary_alpha * attitude_.pitch + 
                            (1.0f - params_.complementary_alpha) * accel_attitude.pitch;
        }
        
        // 磁気センサからのヨー角補正
        if (mag != nullptr) {
            float mag_yaw = estimateYawFromMagnetometer(*mag);
            attitude_.yaw = params_.complementary_alpha * attitude_.yaw + 
                          (1.0f - params_.complementary_alpha) * mag_yaw;
        }
        
        // 角速度更新
        attitude_.roll_rate = corrected_gx;
        attitude_.pitch_rate = corrected_gy;
        attitude_.yaw_rate = corrected_gz;
        
        // 角度正規化
        normalizeAngles();
        
        // ジャイロバイアスの適応的補正
        updateGyroBias(accel_attitude, dt);
    }
    
private:
    // 加速度センサからの初期化
    void initializeFromAccelerometer(const IMUData& imu) {
        attitude_.roll = std::atan2(imu.accel_y, imu.accel_z);
        attitude_.pitch = std::atan2(-imu.accel_x, 
                                    std::sqrt(imu.accel_y*imu.accel_y + imu.accel_z*imu.accel_z));
        attitude_.yaw = 0.0f;  // 初期ヨー角は0と仮定
        is_initialized_ = true;
    }
    
    // 加速度センサからの姿勢推定
    AttitudeData estimateFromAccelerometer(const IMUData& imu) const {
        AttitudeData result;
        result.roll = std::atan2(imu.accel_y, imu.accel_z);
        result.pitch = std::atan2(-imu.accel_x, 
                                 std::sqrt(imu.accel_y*imu.accel_y + imu.accel_z*imu.accel_z));
        return result;
    }
    
    // 磁気センサからのヨー角推定
    float estimateYawFromMagnetometer(const MagnetometerData& mag) const {
        // ティルト補正を適用した磁気コンパス
        float cos_roll = std::cos(attitude_.roll);
        float sin_roll = std::sin(attitude_.roll);
        float cos_pitch = std::cos(attitude_.pitch);
        float sin_pitch = std::sin(attitude_.pitch);
        
        float mag_x_comp = mag.mag_x * cos_pitch + mag.mag_z * sin_pitch;
        float mag_y_comp = mag.mag_x * sin_roll * sin_pitch + 
                          mag.mag_y * cos_roll - 
                          mag.mag_z * sin_roll * cos_pitch;
        
        float yaw = std::atan2(-mag_y_comp, mag_x_comp) + params_.mag_declination;
        return yaw;
    }
    
    // 加速度センサの信頼性判定
    bool isAccelerometerReliable(const IMUData& imu) const {
        float accel_magnitude = std::sqrt(imu.accel_x*imu.accel_x + 
                                         imu.accel_y*imu.accel_y + 
                                         imu.accel_z*imu.accel_z);
        
        float deviation = std::abs(accel_magnitude - gravity_magnitude_);
        return deviation < params_.accel_noise_threshold;
    }
    
    // ジャイロバイアスの適応的補正
    void updateGyroBias(const AttitudeData& accel_attitude, float dt) {
        if (!isAccelerometerReliable({0,0,0,0,0,0})) return;
        
        float roll_error = attitude_.roll - accel_attitude.roll;
        float pitch_error = attitude_.pitch - accel_attitude.pitch;
        
        gyro_bias_x_ += roll_error * params_.gyro_drift_compensation * dt;
        gyro_bias_y_ += pitch_error * params_.gyro_drift_compensation * dt;
        
        // バイアスの制限
        gyro_bias_x_ = std::clamp(gyro_bias_x_, -0.1f, 0.1f);
        gyro_bias_y_ = std::clamp(gyro_bias_y_, -0.1f, 0.1f);
        gyro_bias_z_ = std::clamp(gyro_bias_z_, -0.1f, 0.1f);
    }
    
    // 角度正規化
    void normalizeAngles() {
        attitude_.roll = normalizeAngle(attitude_.roll);
        attitude_.pitch = normalizeAngle(attitude_.pitch);
        attitude_.yaw = normalizeAngle(attitude_.yaw);
    }
    
    float normalizeAngle(float angle) const {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
    
public:
    // 取得関数
    const AttitudeData& getAttitude() const {
        return attitude_;
    }
    
    CoordinateTransform::EulerAngles getEulerAngles() const {
        return CoordinateTransform::EulerAngles(attitude_.roll, attitude_.pitch, attitude_.yaw);
    }
    
    // 設定関数
    void setFilterParameters(const FilterParameters& params) {
        params_ = params;
    }
    
    void resetAttitude() {
        attitude_ = AttitudeData();
        gyro_bias_x_ = gyro_bias_y_ = gyro_bias_z_ = 0.0f;
        is_initialized_ = false;
    }
    
    // キャリブレーション関数
    void calibrateGyroBias(const std::vector<IMUData>& static_samples) {
        if (static_samples.empty()) return;
        
        float sum_gx = 0, sum_gy = 0, sum_gz = 0;
        for (const auto& sample : static_samples) {
            sum_gx += sample.gyro_x;
            sum_gy += sample.gyro_y;
            sum_gz += sample.gyro_z;
        }
        
        gyro_bias_x_ = sum_gx / static_samples.size();
        gyro_bias_y_ = sum_gy / static_samples.size();
        gyro_bias_z_ = sum_gz / static_samples.size();
    }
};
```

## まとめ

本章では、マルチコプタの飛行原理と航空工学の基本をC++オブジェクト指向設計で実装しました。主な成果：

### 実装したクラス体系：

1. **推力計算システム**
   - `ThrustCalculator`: プロペラ推力の精密計算
   - `PropellerPerformanceAnalyzer`: 性能解析と最適化

2. **力学システム**
   - `MulticopterForces`: 重力、推力、空気抗力の管理
   - `MulticopterMoments`: ロール・ピッチ・ヨーモーメント計算

3. **座標変換システム**
   - `CoordinateTransform`: 機体座標系と地球座標系間の安全な変換
   - `RotationMatrix`: 3x3回転行列のカプセル化

4. **モータ制御システム**
   - `MotorController`: 基本的なモータミキシング
   - `AdvancedMotorMixer`: アンチワインドアップとキャリブレーション

5. **環境モデリング**
   - `EnvironmentalFactors`: 空気密度、地面効果、温度補正
   - `AdvancedAttitudeEstimator`: センサフュージョンと姿勢推定

### 設計の特徴：

- **モジュラー設計**: 各クラスは独立してテスト・使用可能
- **型安全性**: コンパイル時エラー検出と適切な型チェック
- **パフォーマンス**: リアルタイム制御に必要な計算効率
- **保守性**: 入力検証、範囲制限、エラーハンドリング
- **拡張性**: 継承とパラメータカスタマイズによる柔軟性

### 実用的な利点：

1. **StampFly統合**: ESP-IDF環境での直接利用が可能
2. **デバッグ容易性**: クラス単位での独立テストと検証
3. **メンテナンス性**: 明確な責任分担とインターフェース
4. **教育価値**: 各クラスが航空工学の特定概念を明確に表現

### 次章への連携：

これらの航空工学クラスは、第3章の剛体運動クラスと結合して、完全なマルチコプタシミュレーションシステムを構成します。特に：

- **推力計算** → 第3章の`MulticopterDynamics`と連携
- **座標変換** → 第3章の`RotationMatrix`と統合
- **モータ制御** → 第4章の制御アルゴリズムと結合
- **環境モデル** → 第5章の実機制御で活用

次章では、これらの航空工学クラスを基盤として、6自由度の剛体運動を数学的にモデル化し、統合されたシミュレーションシステムを構築していきます。

## 参考文献

### 書籍・論文
- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice
- Quan, Q. (2017). Introduction to Multicopter Design and Control
- Mahony, R., Kumar, V., & Corke, P. (2012). Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor

### 実践的資料・発表資料
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343) - 102.2K views
- [StampFlyで学ぶマルチコプタ制御](https://www.docswell.com/s/Kouhei_Ito/K38V1P-2024-02-10-094123) - 314.3K views
- [2025StampFly勉強会](https://www.docswell.com/s/Kouhei_Ito/K4VR7G-2025-03-23-104258) - 57.3K views

---

*本記事は「StampFly制御システム完全ガイド」シリーズの第2章です。*