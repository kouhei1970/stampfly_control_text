# 第3章 剛体の運動 - StampFly制御システム入門

## はじめに

前章では、マルチコプタの飛行原理と航空工学の基本を学びました。本章では、マルチコプタを剛体として扱い、その運動を数学的にモデル化する方法を詳しく学びます。このモデルは、制御システムを設計する上で不可欠な基礎となります。

## 3.1 6自由度運動

3次元空間における剛体の運動は、6つの自由度で完全に記述できます。

### 3.1.1 並進運動（3自由度）

位置ベクトル **r** = [x, y, z]ᵀ で表される：

- **x**: 前後方向の位置
- **y**: 左右方向の位置
- **z**: 上下方向の位置

### 3.1.2 回転運動（3自由度）

姿勢を表す3つの角度：

- **φ (Roll)**: X軸周りの回転
- **θ (Pitch)**: Y軸周りの回転
- **ψ (Yaw)**: Z軸周りの回転

### 3.1.3 状態ベクトル

マルチコプタの完全な状態は12次元のベクトルで表現されます：

```cpp
class MulticopterState {
public:
    // 位置情報（地球座標系）
    struct Position {
        float x, y, z;  // [m]
        
        Position(float x = 0.0f, float y = 0.0f, float z = 0.0f) 
            : x(x), y(y), z(z) {}
    } position;
    
    // 機体軸座標系での速度
    struct BodyVelocity {
        float u, v, w;  // 前進・横・上下速度 [m/s]
        
        BodyVelocity(float u = 0.0f, float v = 0.0f, float w = 0.0f) 
            : u(u), v(v), w(w) {}
    } body_velocity;
    
    // 姿勢（オイラー角）
    struct Attitude {
        float roll, pitch, yaw;  // [rad]
        
        Attitude(float roll = 0.0f, float pitch = 0.0f, float yaw = 0.0f)
            : roll(roll), pitch(pitch), yaw(yaw) {}
    } attitude;
    
    // 機体軸座標系での角速度
    struct BodyAngularVelocity {
        float p, q, r;  // roll rate, pitch rate, yaw rate [rad/s]
        
        BodyAngularVelocity(float p = 0.0f, float q = 0.0f, float r = 0.0f)
            : p(p), q(q), r(r) {}
    } body_angular_velocity;
    
    // コンストラクタ
    MulticopterState() = default;
    
    // 状態をリセット
    void reset() {
        position = Position();
        body_velocity = BodyVelocity();
        attitude = Attitude();
        body_angular_velocity = BodyAngularVelocity();
    }
    
    // 状態の有効性チェック
    bool isValid() const {
        // NaNや無限大値のチェック
        return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
               std::isfinite(body_velocity.u) && std::isfinite(body_velocity.v) && std::isfinite(body_velocity.w) &&
               std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) && std::isfinite(attitude.yaw) &&
               std::isfinite(body_angular_velocity.p) && std::isfinite(body_angular_velocity.q) && std::isfinite(body_angular_velocity.r);
    }
};
```

## 3.2 運動方程式の導出

### 3.2.1 ニュートンの運動方程式（並進運動）

質量中心の並進運動は、ニュートンの第2法則により：

```
m × dV/dt = ΣF
```

ここで：
- **m**: 機体質量
- **V**: 速度ベクトル
- **ΣF**: 外力の総和

機体軸座標系でのオブジェクト指向設計：

```cpp
class TranslationalDynamics {
public:
    // 機体パラメータ
    struct Parameters {
        static constexpr float GRAVITY = 9.81f;  // [m/s^2]
        float mass;  // 機体質量 [kg]
        
        explicit Parameters(float mass) : mass(mass) {}
    };
    
    // 外力ベクトル（機体軸座標系）
    struct BodyForces {
        float fx, fy, fz;  // [N]
        
        BodyForces(float fx = 0.0f, float fy = 0.0f, float fz = 0.0f)
            : fx(fx), fy(fy), fz(fz) {}
    };
    
    // 機体軸加速度計算
    struct BodyAcceleration {
        float ax, ay, az;  // [m/s^2]
        
        BodyAcceleration(float ax = 0.0f, float ay = 0.0f, float az = 0.0f)
            : ax(ax), ay(ay), az(az) {}
    };
    
private:
    Parameters params_;
    
public:
    explicit TranslationalDynamics(const Parameters& params) : params_(params) {}
    
    // 機体軸座標系での並進運動方程式
    BodyAcceleration computeBodyAcceleration(
        const MulticopterState& state,
        float thrust,
        const RotationMatrix& rotation_matrix) const {
        
        // 機体軸座標系での外力
        BodyForces forces;
        forces.fx = 0.0f;
        forces.fy = 0.0f;
        forces.fz = -thrust;  // 推力は機体のZ軸負方向
        
        // 重力を地球座標系から機体座標系に変換
        const auto gravity_body = rotation_matrix.earthToBody({0.0f, 0.0f, Parameters::GRAVITY});
        forces.fx += gravity_body.x * params_.mass;
        forces.fy += gravity_body.y * params_.mass;
        forces.fz += gravity_body.z * params_.mass;
        
        // コリオリ効果と遠心力効果を含む機体軸加速度
        const auto& vel = state.body_velocity;
        const auto& omega = state.body_angular_velocity;
        
        BodyAcceleration accel;
        accel.ax = forces.fx / params_.mass + omega.r * vel.v - omega.q * vel.w;
        accel.ay = forces.fy / params_.mass + omega.p * vel.w - omega.r * vel.u;
        accel.az = forces.fz / params_.mass + omega.q * vel.u - omega.p * vel.v;
        
        return accel;
    }
};
```

### 3.2.3 座標系変換の重要性

機体軸座標系での記述により：

**利点：**
- 制御入力（推力・モーメント）が直接機体軸で扱える
- コリオリ効果・遠心力効果が自然に表現される
- 非線形項が物理的に意味のある形で現れる
- 制御系設計が直感的になる

**モダンC++での実装特徴：**
```cpp
// 型安全性とカプセル化
class TranslationalDynamics {
    static constexpr float GRAVITY = 9.81f;  // コンパイル時定数
    
public:
    // 明示的なコンストラクタで初期化を強制
    explicit TranslationalDynamics(const Parameters& params);
    
    // const正確性と参照渡しでパフォーマンス最適化
    BodyAcceleration computeBodyAcceleration(
        const MulticopterState& state,
        float thrust,
        const RotationMatrix& rotation_matrix) const;
};
```

### 3.2.4 オイラーの運動方程式（回転運動）

剛体の回転運動は、オイラーの運動方程式により：

```
I × dω/dt + ω × (I × ω) = ΣM
```

ここで：
- **I**: 慣性テンソル
- **ω**: 角速度ベクトル
- **ΣM**: 外部モーメントの総和

機体座標系でのオブジェクト指向実装：

```cpp
class RotationalDynamics {
public:
    // 慣性テンソル（対角化済み）
    struct InertiaMatrix {
        float Ixx, Iyy, Izz;  // [kg⋅m²]
        float Ixy = 0.0f, Ixz = 0.0f, Iyz = 0.0f;  // 慣性積（通常0）
        
        InertiaMatrix(float Ixx, float Iyy, float Izz)
            : Ixx(Ixx), Iyy(Iyy), Izz(Izz) {}
        
        // StampFly用デフォルト値
        static InertiaMatrix createStampFlyDefault() {
            return InertiaMatrix(0.01f, 0.01f, 0.02f);
        }
    };
    
    // 外部モーメント
    struct BodyMoments {
        float Mx, My, Mz;  // [N⋅m]
        
        BodyMoments(float Mx = 0.0f, float My = 0.0f, float Mz = 0.0f)
            : Mx(Mx), My(My), Mz(Mz) {}
    };
    
    // 角加速度
    struct AngularAcceleration {
        float alpha_x, alpha_y, alpha_z;  // [rad/s²]
        
        AngularAcceleration(float ax = 0.0f, float ay = 0.0f, float az = 0.0f)
            : alpha_x(ax), alpha_y(ay), alpha_z(az) {}
    };
    
private:
    InertiaMatrix inertia_;
    
public:
    explicit RotationalDynamics(const InertiaMatrix& inertia) : inertia_(inertia) {}
    
    // オイラーの運動方程式
    AngularAcceleration computeAngularAcceleration(
        const MulticopterState& state,
        const BodyMoments& moments) const {
        
        const auto& omega = state.body_angular_velocity;
        
        // ジャイロ効果項（ω × (I × ω)）
        const float gyro_x = omega.q * omega.r * (inertia_.Iyy - inertia_.Izz);
        const float gyro_y = omega.r * omega.p * (inertia_.Izz - inertia_.Ixx);
        const float gyro_z = omega.p * omega.q * (inertia_.Ixx - inertia_.Iyy);
        
        // 角加速度の計算
        AngularAcceleration alpha;
        alpha.alpha_x = (moments.Mx - gyro_x) / inertia_.Ixx;
        alpha.alpha_y = (moments.My - gyro_y) / inertia_.Iyy;
        alpha.alpha_z = (moments.Mz - gyro_z) / inertia_.Izz;
        
        return alpha;
    }
    
    // 慣性テンソルの取得
    const InertiaMatrix& getInertiaMatrix() const { return inertia_; }
};
```

## 3.3 姿勢の表現方法

### 3.3.1 オイラー角クラス

最も直感的な姿勢表現ですが、特異点（ジンバルロック）の問題があります。オブジェクト指向で実装：

```cpp
class EulerAngles {
public:
    float roll, pitch, yaw;  // [rad]
    
    EulerAngles(float roll = 0.0f, float pitch = 0.0f, float yaw = 0.0f)
        : roll(roll), pitch(pitch), yaw(yaw) {}
    
    // 角度の正規化
    void normalize() {
        roll = normalizeAngle(roll);
        pitch = normalizeAngle(pitch);
        yaw = normalizeAngle(yaw);
    }
    
    // 特異点チェック
    bool hasSingularity(float tolerance = 0.01f) const {
        return std::abs(std::cos(pitch)) < tolerance;
    }
    
    // 度数表現への変換
    struct Degrees {
        float roll, pitch, yaw;
    };
    
    Degrees toDegrees() const {
        constexpr float RAD_TO_DEG = 180.0f / M_PI;
        return {roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG};
    }
    
private:
    static float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
};
```

#### オイラー角の微分方程式クラス

角速度とオイラー角の変化率の関係を管理するクラス：

```cpp
class EulerAngleIntegrator {
public:
    // 角度変化率
    struct AngleRates {
        float roll_rate, pitch_rate, yaw_rate;  // [rad/s]
        
        AngleRates(float roll_rate = 0.0f, float pitch_rate = 0.0f, float yaw_rate = 0.0f)
            : roll_rate(roll_rate), pitch_rate(pitch_rate), yaw_rate(yaw_rate) {}
    };
    
    // 角速度からオイラー角変化率を計算
    static AngleRates computeAngleRates(
        const EulerAngles& angles,
        const MulticopterState::BodyAngularVelocity& body_omega) {
        
        const float phi = angles.roll;
        const float theta = angles.pitch;
        const float cos_theta = std::cos(theta);
        const float sin_phi = std::sin(phi);
        const float cos_phi = std::cos(phi);
        const float tan_theta = std::tan(theta);
        
        // 特異点チェック
        if (std::abs(cos_theta) < 0.01f) {
            // 特異点付近では変化率を0に設定
            return AngleRates();
        }
        
        AngleRates rates;
        rates.roll_rate = body_omega.p + body_omega.q * sin_phi * tan_theta + 
                         body_omega.r * cos_phi * tan_theta;
        rates.pitch_rate = body_omega.q * cos_phi - body_omega.r * sin_phi;
        rates.yaw_rate = (body_omega.q * sin_phi + body_omega.r * cos_phi) / cos_theta;
        
        return rates;
    }
    
    // オイラー角の数値積分
    static EulerAngles integrate(
        const EulerAngles& current_angles,
        const MulticopterState::BodyAngularVelocity& body_omega,
        float dt) {
        
        const auto rates = computeAngleRates(current_angles, body_omega);
        
        EulerAngles new_angles;
        new_angles.roll = current_angles.roll + rates.roll_rate * dt;
        new_angles.pitch = current_angles.pitch + rates.pitch_rate * dt;
        new_angles.yaw = current_angles.yaw + rates.yaw_rate * dt;
        
        new_angles.normalize();
        return new_angles;
    }
};
```

### 3.3.2 四元数（Quaternion）クラス

特異点がなく、計算効率の良い姺勢表現をオブジェクト指向で実装：

```cpp
class Quaternion {
public:
    float w, x, y, z;  // w:スカラー部、x,y,z:ベクトル部
    
    // コンストラクタ
    Quaternion(float w = 1.0f, float x = 0.0f, float y = 0.0f, float z = 0.0f)
        : w(w), x(x), y(y), z(z) {}
    
    // オイラー角からの構築
    static Quaternion fromEulerAngles(const EulerAngles& euler) {
        const float cr = std::cos(euler.roll * 0.5f);
        const float sr = std::sin(euler.roll * 0.5f);
        const float cp = std::cos(euler.pitch * 0.5f);
        const float sp = std::sin(euler.pitch * 0.5f);
        const float cy = std::cos(euler.yaw * 0.5f);
        const float sy = std::sin(euler.yaw * 0.5f);
        
        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        
        return q;
    }
    
    // 正規化
    void normalize() {
        const float norm = std::sqrt(w*w + x*x + y*y + z*z);
        if (norm > 1e-6f) {
            const float inv_norm = 1.0f / norm;
            w *= inv_norm;
            x *= inv_norm;
            y *= inv_norm;
            z *= inv_norm;
        }
    }
    
    // 正規化された四元数を返す
    Quaternion normalized() const {
        Quaternion result = *this;
        result.normalize();
        return result;
    }
    
    // 共役
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // ノルム
    float norm() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }
    
    // 四元数の乗算
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }
    
    // オイラー角への変換
    EulerAngles toEulerAngles() const {
        EulerAngles euler;
        
        // ロール
        const float sinr_cosp = 2.0f * (w * x + y * z);
        const float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        euler.roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // ピッチ
        const float sinp = 2.0f * (w * y - z * x);
        if (std::abs(sinp) >= 1.0f)
            euler.pitch = std::copysign(M_PI / 2.0f, sinp);  // 特異点
        else
            euler.pitch = std::asin(sinp);
        
        // ヨー
        const float siny_cosp = 2.0f * (w * z + x * y);
        const float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        euler.yaw = std::atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
};
```

#### 四元数の積分器クラス

```cpp
class QuaternionIntegrator {
public:
    // 四元数の微分
    struct QuaternionDerivative {
        float dw, dx, dy, dz;
        
        QuaternionDerivative(float dw = 0.0f, float dx = 0.0f, float dy = 0.0f, float dz = 0.0f)
            : dw(dw), dx(dx), dy(dy), dz(dz) {}
    };
    
    // 角速度から四元数微分を計算
    static QuaternionDerivative computeDerivative(
        const Quaternion& q,
        const MulticopterState::BodyAngularVelocity& omega) {
        
        // 四元数の微分方程式: dq/dt = 0.5 * q * ω
        QuaternionDerivative dq;
        dq.dw = 0.5f * (-omega.p * q.x - omega.q * q.y - omega.r * q.z);
        dq.dx = 0.5f * (omega.p * q.w + omega.r * q.y - omega.q * q.z);
        dq.dy = 0.5f * (omega.q * q.w - omega.r * q.x + omega.p * q.z);
        dq.dz = 0.5f * (omega.r * q.w + omega.q * q.x - omega.p * q.y);
        
        return dq;
    }
    
    // オイラー法による数値積分
    static Quaternion integrate(
        const Quaternion& current_q,
        const MulticopterState::BodyAngularVelocity& omega,
        float dt) {
        
        const auto dq = computeDerivative(current_q, omega);
        
        Quaternion new_q;
        new_q.w = current_q.w + dq.dw * dt;
        new_q.x = current_q.x + dq.dx * dt;
        new_q.y = current_q.y + dq.dy * dt;
        new_q.z = current_q.z + dq.dz * dt;
        
        // 正規化（数値誤差の蓄積を防ぐ）
        new_q.normalize();
        return new_q;
    }
    
    // 高精度ルンゲ・クッタ法
    static Quaternion integrateRK4(
        const Quaternion& current_q,
        const MulticopterState::BodyAngularVelocity& omega,
        float dt) {
        
        // k1
        const auto k1 = computeDerivative(current_q, omega);
        
        // k2
        Quaternion q2(current_q.w + k1.dw * dt * 0.5f,
                     current_q.x + k1.dx * dt * 0.5f,
                     current_q.y + k1.dy * dt * 0.5f,
                     current_q.z + k1.dz * dt * 0.5f);
        const auto k2 = computeDerivative(q2, omega);
        
        // k3
        Quaternion q3(current_q.w + k2.dw * dt * 0.5f,
                     current_q.x + k2.dx * dt * 0.5f,
                     current_q.y + k2.dy * dt * 0.5f,
                     current_q.z + k2.dz * dt * 0.5f);
        const auto k3 = computeDerivative(q3, omega);
        
        // k4
        Quaternion q4(current_q.w + k3.dw * dt,
                     current_q.x + k3.dx * dt,
                     current_q.y + k3.dy * dt,
                     current_q.z + k3.dz * dt);
        const auto k4 = computeDerivative(q4, omega);
        
        // 重み付け平均
        Quaternion result;
        result.w = current_q.w + (k1.dw + 2.0f * k2.dw + 2.0f * k3.dw + k4.dw) * dt / 6.0f;
        result.x = current_q.x + (k1.dx + 2.0f * k2.dx + 2.0f * k3.dx + k4.dx) * dt / 6.0f;
        result.y = current_q.y + (k1.dy + 2.0f * k2.dy + 2.0f * k3.dy + k4.dy) * dt / 6.0f;
        result.z = current_q.z + (k1.dz + 2.0f * k2.dz + 2.0f * k3.dz + k4.dz) * dt / 6.0f;
        
        result.normalize();
        return result;
    }
};
```

### 3.3.3 回転行列クラス

座標変換に直接使用できる表現をオブジェクト指向で実装：

```cpp
class RotationMatrix {
public:
    // 3x3行列要素
    struct Matrix3x3 {
        float data[3][3];
        
        // デフォルトコンストラクタ（単位行列）
        Matrix3x3() {
            data[0][0] = 1.0f; data[0][1] = 0.0f; data[0][2] = 0.0f;
            data[1][0] = 0.0f; data[1][1] = 1.0f; data[1][2] = 0.0f;
            data[2][0] = 0.0f; data[2][1] = 0.0f; data[2][2] = 1.0f;
        }
        
        // 行へのアクセス
        float* operator[](int row) { return data[row]; }
        const float* operator[](int row) const { return data[row]; }
    };
    
private:
    Matrix3x3 matrix_;
    
public:
    RotationMatrix() = default;
    
    // オイラー角からの構築
    static RotationMatrix fromEulerAngles(const EulerAngles& euler) {
        const float cr = std::cos(euler.roll);
        const float sr = std::sin(euler.roll);
        const float cp = std::cos(euler.pitch);
        const float sp = std::sin(euler.pitch);
        const float cy = std::cos(euler.yaw);
        const float sy = std::sin(euler.yaw);
        
        RotationMatrix R;
        R.matrix_[0][0] = cp * cy;
        R.matrix_[0][1] = sr * sp * cy - cr * sy;
        R.matrix_[0][2] = cr * sp * cy + sr * sy;
        
        R.matrix_[1][0] = cp * sy;
        R.matrix_[1][1] = sr * sp * sy + cr * cy;
        R.matrix_[1][2] = cr * sp * sy - sr * cy;
        
        R.matrix_[2][0] = -sp;
        R.matrix_[2][1] = sr * cp;
        R.matrix_[2][2] = cr * cp;
        
        return R;
    }
    
    // 四元数からの構築
    static RotationMatrix fromQuaternion(const Quaternion& q) {
        const float w = q.w, x = q.x, y = q.y, z = q.z;
        
        RotationMatrix R;
        R.matrix_[0][0] = 1.0f - 2.0f * (y*y + z*z);
        R.matrix_[0][1] = 2.0f * (x*y - w*z);
        R.matrix_[0][2] = 2.0f * (x*z + w*y);
        
        R.matrix_[1][0] = 2.0f * (x*y + w*z);
        R.matrix_[1][1] = 1.0f - 2.0f * (x*x + z*z);
        R.matrix_[1][2] = 2.0f * (y*z - w*x);
        
        R.matrix_[2][0] = 2.0f * (x*z - w*y);
        R.matrix_[2][1] = 2.0f * (y*z + w*x);
        R.matrix_[2][2] = 1.0f - 2.0f * (x*x + y*y);
        
        return R;
    }
    
    // ベクトル変換
    struct Vector3 {
        float x, y, z;
        
        Vector3(float x = 0.0f, float y = 0.0f, float z = 0.0f)
            : x(x), y(y), z(z) {}
    };
    
    // 機体座標系から地球座標系へ
    Vector3 bodyToEarth(const Vector3& v_body) const {
        return Vector3(
            matrix_[0][0] * v_body.x + matrix_[0][1] * v_body.y + matrix_[0][2] * v_body.z,
            matrix_[1][0] * v_body.x + matrix_[1][1] * v_body.y + matrix_[1][2] * v_body.z,
            matrix_[2][0] * v_body.x + matrix_[2][1] * v_body.y + matrix_[2][2] * v_body.z
        );
    }
    
    // 地球座標系から機体座標系へ（転置行列を使用）
    Vector3 earthToBody(const Vector3& v_earth) const {
        return Vector3(
            matrix_[0][0] * v_earth.x + matrix_[1][0] * v_earth.y + matrix_[2][0] * v_earth.z,
            matrix_[0][1] * v_earth.x + matrix_[1][1] * v_earth.y + matrix_[2][1] * v_earth.z,
            matrix_[0][2] * v_earth.x + matrix_[1][2] * v_earth.y + matrix_[2][2] * v_earth.z
        );
    }
    
    // 行列要素へのアクセス
    const Matrix3x3& getMatrix() const { return matrix_; }
};
```

## 3.4 慣性モーメント管理クラス

### 3.4.1 慣性テンソルクラス

剛体の回転に対する慣性を管理するクラス：

```cpp
class InertiaMatrix {
public:
    // 慣性テンソル成分
    struct Components {
        float Ixx, Iyy, Izz;         // 対角成分 [kg⋅m²]
        float Ixy = 0.0f, Ixz = 0.0f, Iyz = 0.0f;  // 慣性積（通常0）
        
        Components(float Ixx = 0.01f, float Iyy = 0.01f, float Izz = 0.02f)
            : Ixx(Ixx), Iyy(Iyy), Izz(Izz) {}
    };
    
private:
    Components components_;
    
public:
    explicit InertiaMatrix(const Components& components = Components())
        : components_(components) {}
    
    // StampFlyデフォルト値
    static InertiaMatrix createStampFlyDefault() {
        return InertiaMatrix(Components(0.01f, 0.01f, 0.02f));
    }
    
    // プロペティ取得
    float getIxx() const { return components_.Ixx; }
    float getIyy() const { return components_.Iyy; }
    float getIzz() const { return components_.Izz; }
    
    // 質点系からの計算
    static InertiaMatrix fromMassDistribution(
        float motor_mass,   // 各モータ質量 [kg]
        float arm_length) { // アーム長 [m]
        
        // X配置の場合、各モータは中心から45度の位置
        const float r = arm_length / std::sqrt(2.0f);  // 各軸への投影距離
        
        Components comp;
        comp.Ixx = 2.0f * motor_mass * r * r;  // 前後のモータペア
        comp.Iyy = 2.0f * motor_mass * r * r;  // 左右のモータペア
        comp.Izz = 4.0f * motor_mass * r * r;  // 全モータ
        
        return InertiaMatrix(comp);
    }
    
    // ジャイロ効果計算
    RotationMatrix::Vector3 computeGyroscopicEffect(
        const MulticopterState::BodyAngularVelocity& omega) const {
        
        return RotationMatrix::Vector3(
            omega.q * omega.r * (components_.Iyy - components_.Izz),
            omega.r * omega.p * (components_.Izz - components_.Ixx),
            omega.p * omega.q * (components_.Ixx - components_.Iyy)
        );
    }
};
```

### 3.4.2 慣性モーメントの実験的同定

振り子実験による同定クラス：

```cpp
class InertiaIdentification {
public:
    // 振り子実験データ
    struct PendulumData {
        float period;           // 振動周期 [s]
        float pivot_distance;   // 回転軸から重心までの距離 [m]
        float mass;            // 機体質量 [kg]
        
        PendulumData(float period, float pivot_distance, float mass)
            : period(period), pivot_distance(pivot_distance), mass(mass) {}
    };
    
    // 複合振り子法による慣性モーメント同定
    static float identifyFromPendulum(const PendulumData& data) {
        // T = 2π√(I/(m*g*d))
        const float g = 9.81f;  // 重力加速度
        const float I = (data.period * data.period * data.mass * g * data.pivot_distance) / 
                       (4.0f * M_PI * M_PI);
        return I;
    }
    
    // 複数軸での実験から完全な慣性テンソルを同定
    static InertiaMatrix identifyFullMatrix(
        const PendulumData& roll_data,
        const PendulumData& pitch_data,
        const PendulumData& yaw_data) {
        
        InertiaMatrix::Components comp;
        comp.Ixx = identifyFromPendulum(pitch_data);  // ピッチ軸回りの振動
        comp.Iyy = identifyFromPendulum(roll_data);   // ロール軸回りの振動
        comp.Izz = identifyFromPendulum(yaw_data);    // ヨー軸回りの振動
        
        return InertiaMatrix(comp);
    }
    
    // 動的同定（飛行中のデータから）
    static InertiaMatrix identifyFromFlightData(
        const std::vector<MulticopterState>& states,
        const std::vector<RotationalDynamics::BodyMoments>& moments,
        float dt) {
        
        // 最小二乗法で慣性モーメントを推定
        // TODO: 実装は省略（高度な最適化手法が必要）
        return InertiaMatrix::createStampFlyDefault();
    }
};
```

## 3.5 座標変換管理クラス

### 3.5.1 統合座標変換クラス

各種座標変換を統一的に管理するクラス：

```cpp
class CoordinateTransform {
private:
    RotationMatrix rotation_matrix_;
    Quaternion quaternion_;
    EulerAngles euler_angles_;
    
    // 内部一貫性フラグ
    mutable bool rotation_matrix_valid_ = false;
    mutable bool quaternion_valid_ = false;
    mutable bool euler_angles_valid_ = false;
    
public:
    // オイラー角から構築
    explicit CoordinateTransform(const EulerAngles& euler)
        : euler_angles_(euler), euler_angles_valid_(true) {}
    
    // 四元数から構築
    explicit CoordinateTransform(const Quaternion& quat)
        : quaternion_(quat), quaternion_valid_(true) {}
    
    // 回転行列を取得（必要に応じて計算）
    const RotationMatrix& getRotationMatrix() const {
        if (!rotation_matrix_valid_) {
            if (quaternion_valid_) {
                rotation_matrix_ = RotationMatrix::fromQuaternion(quaternion_);
            } else if (euler_angles_valid_) {
                rotation_matrix_ = RotationMatrix::fromEulerAngles(euler_angles_);
            }
            rotation_matrix_valid_ = true;
        }
        return rotation_matrix_;
    }
    
    // 四元数を取得（必要に応じて計算）
    const Quaternion& getQuaternion() const {
        if (!quaternion_valid_) {
            if (euler_angles_valid_) {
                quaternion_ = Quaternion::fromEulerAngles(euler_angles_);
            } else if (rotation_matrix_valid_) {
                // TODO: 回転行列から四元数への変換実装
            }
            quaternion_valid_ = true;
        }
        return quaternion_;
    }
    
    // ベクトル変換
    RotationMatrix::Vector3 bodyToEarth(const RotationMatrix::Vector3& v_body) const {
        return getRotationMatrix().bodyToEarth(v_body);
    }
    
    RotationMatrix::Vector3 earthToBody(const RotationMatrix::Vector3& v_earth) const {
        return getRotationMatrix().earthToBody(v_earth);
    }
};
```

### 3.5.2 角速度変換クラス

機体座標系の角速度から地球座標系のオイラー角変化率への変換を管理：

```cpp
class AngularVelocityTransform {
public:
    // 変換行列
    struct TransformMatrix {
        float data[3][3];
        
        float* operator[](int row) { return data[row]; }
        const float* operator[](int row) const { return data[row]; }
    };
    
    // オイラー角から変換行列を計算
    static TransformMatrix computeTransformMatrix(const EulerAngles& euler) {
        const float sp = std::sin(euler.roll);
        const float cp = std::cos(euler.roll);
        const float st = std::sin(euler.pitch);
        const float ct = std::cos(euler.pitch);
        const float tt = std::tan(euler.pitch);
        
        TransformMatrix T;
        T[0][0] = 1.0f;  T[0][1] = sp * tt;  T[0][2] = cp * tt;
        T[1][0] = 0.0f;  T[1][1] = cp;       T[1][2] = -sp;
        T[2][0] = 0.0f;  T[2][1] = sp / ct;  T[2][2] = cp / ct;
        
        return T;
    }
    
    // 機体軸角速度からオイラー角変化率へ
    static EulerAngleIntegrator::AngleRates bodyToEulerRates(
        const MulticopterState::BodyAngularVelocity& body_omega,
        const EulerAngles& euler) {
        
        const auto T = computeTransformMatrix(euler);
        
        EulerAngleIntegrator::AngleRates rates;
        rates.roll_rate = T[0][0] * body_omega.p + T[0][1] * body_omega.q + T[0][2] * body_omega.r;
        rates.pitch_rate = T[1][0] * body_omega.p + T[1][1] * body_omega.q + T[1][2] * body_omega.r;
        rates.yaw_rate = T[2][0] * body_omega.p + T[2][1] * body_omega.q + T[2][2] * body_omega.r;
        
        return rates;
    }
    
    // オイラー角変化率から機体軸角速度へ（逆変換）
    static MulticopterState::BodyAngularVelocity eulerRatesToBody(
        const EulerAngleIntegrator::AngleRates& euler_rates,
        const EulerAngles& euler) {
        
        const float sp = std::sin(euler.roll);
        const float cp = std::cos(euler.roll);
        const float st = std::sin(euler.pitch);
        const float ct = std::cos(euler.pitch);
        
        MulticopterState::BodyAngularVelocity omega;
        omega.p = euler_rates.roll_rate - st * euler_rates.yaw_rate;
        omega.q = cp * euler_rates.pitch_rate + sp * ct * euler_rates.yaw_rate;
        omega.r = -sp * euler_rates.pitch_rate + cp * ct * euler_rates.yaw_rate;
        
        return omega;
    }
};
```

## 3.6 完全な運動モデルクラス

### 3.6.1 統合ダイナミクスクラス

マルチコプタの非線形運動方程式を統合的に管理：

```cpp
class MulticopterDynamics {
public:
    // 制御入力
    struct ControlInput {
        float motor_commands[4];  // 各モータのPWM指令 [0-1]
        
        ControlInput() {
            std::fill(std::begin(motor_commands), std::end(motor_commands), 0.0f);
        }
    };
    
    // 状態微分
    struct StateDerivative {
        // 位置微分
        float x_dot, y_dot, z_dot;
        // 機体軸速度微分
        float u_dot, v_dot, w_dot;
        // 姿勢微分
        float roll_dot, pitch_dot, yaw_dot;
        // 角速度微分
        float p_dot, q_dot, r_dot;
    };
    
    // パラメータ
    struct Parameters {
        float mass;                     // 機体質量 [kg]
        float arm_length;              // アーム長 [m]
        float thrust_coefficient;      // 推力係数
        float moment_coefficient;      // モーメント係数
        InertiaMatrix inertia;         // 慣性テンソル
        
        // StampFlyデフォルト
        static Parameters createStampFlyDefault() {
            Parameters params;
            params.mass = 0.0368f;  // 36.8g
            params.arm_length = 0.0815f / std::sqrt(2.0f);  // 81.5mm、X配置
            params.thrust_coefficient = 1.0e-6f;
            params.moment_coefficient = 1.0e-8f;
            params.inertia = InertiaMatrix::createStampFlyDefault();
            return params;
        }
    };
    
private:
    Parameters params_;
    TranslationalDynamics translational_dynamics_;
    RotationalDynamics rotational_dynamics_;
    
public:
    explicit MulticopterDynamics(const Parameters& params)
        : params_(params),
          translational_dynamics_(TranslationalDynamics::Parameters(params.mass)),
          rotational_dynamics_(params.inertia) {}
    
    // 完全な状態微分計算
    StateDerivative computeDerivatives(
        const MulticopterState& state,
        const ControlInput& input) const {
        
        StateDerivative deriv;
        
        // 推力とモーメントの計算
        const auto forces_moments = computeForcesAndMoments(input);
        
        // 姿勢から回転行列を構築
        const auto rotation = RotationMatrix::fromEulerAngles(state.attitude);
        
        // 位置微分（機体軸速度を地球座標系に変換）
        const auto earth_velocity = rotation.bodyToEarth(
            {state.body_velocity.u, state.body_velocity.v, state.body_velocity.w});
        deriv.x_dot = earth_velocity.x;
        deriv.y_dot = earth_velocity.y;
        deriv.z_dot = earth_velocity.z;
        
        // 機体軸速度微分（並進運動）
        const auto body_accel = translational_dynamics_.computeBodyAcceleration(
            state, forces_moments.thrust, rotation);
        deriv.u_dot = body_accel.ax;
        deriv.v_dot = body_accel.ay;
        deriv.w_dot = body_accel.az;
        
        // 姿勢微分（オイラー角変化率）
        const auto angle_rates = EulerAngleIntegrator::computeAngleRates(
            state.attitude, state.body_angular_velocity);
        deriv.roll_dot = angle_rates.roll_rate;
        deriv.pitch_dot = angle_rates.pitch_rate;
        deriv.yaw_dot = angle_rates.yaw_rate;
        
        // 角速度微分（回転運動）
        const auto angular_accel = rotational_dynamics_.computeAngularAcceleration(
            state, forces_moments.moments);
        deriv.p_dot = angular_accel.alpha_x;
        deriv.q_dot = angular_accel.alpha_y;
        deriv.r_dot = angular_accel.alpha_z;
        
        return deriv;
    }
    
private:
    // 推力とモーメントの計算
    struct ForcesAndMoments {
        float thrust;
        RotationalDynamics::BodyMoments moments;
    };
    
    ForcesAndMoments computeForcesAndMoments(const ControlInput& input) const {
        ForcesAndMoments result;
        
        // 各モータの推力計算
        float motor_thrusts[4];
        for (int i = 0; i < 4; ++i) {
            motor_thrusts[i] = params_.thrust_coefficient * 
                              input.motor_commands[i] * input.motor_commands[i];
        }
        
        // 総推力
        result.thrust = motor_thrusts[0] + motor_thrusts[1] + 
                       motor_thrusts[2] + motor_thrusts[3];
        
        // モーメント計算（X配置）
        result.moments.Mx = params_.arm_length * 
            (-motor_thrusts[0] + motor_thrusts[1] + motor_thrusts[2] - motor_thrusts[3]);
        result.moments.My = params_.arm_length * 
            (motor_thrusts[0] + motor_thrusts[1] - motor_thrusts[2] - motor_thrusts[3]);
        result.moments.Mz = params_.moment_coefficient * 
            (-input.motor_commands[0] + input.motor_commands[1] - 
             input.motor_commands[2] + input.motor_commands[3]);
        
        return result;
    }
};
```

### 3.6.2 数値積分クラス

運動方程式の数値解法を扱うクラス：

```cpp
// 4次ルンゲ・クッタ積分器クラス
class RungeKutta4Integrator {
public:
    // 状態微分の定義
    struct StateDerivative {
        float x_dot, y_dot, z_dot;        // 位置微分
        float u_dot, v_dot, w_dot;        // 機体軸速度微分
        float phi_dot, theta_dot, psi_dot; // 姿勢微分
        float p_dot, q_dot, r_dot;        // 角速度微分
        
        StateDerivative() = default;
        
        // スケール演算子
        StateDerivative operator*(float scale) const {
            StateDerivative result;
            result.x_dot = x_dot * scale;
            result.y_dot = y_dot * scale;
            result.z_dot = z_dot * scale;
            result.u_dot = u_dot * scale;
            result.v_dot = v_dot * scale;
            result.w_dot = w_dot * scale;
            result.phi_dot = phi_dot * scale;
            result.theta_dot = theta_dot * scale;
            result.psi_dot = psi_dot * scale;
            result.p_dot = p_dot * scale;
            result.q_dot = q_dot * scale;
            result.r_dot = r_dot * scale;
            return result;
        }
        
        // 加算演算子
        StateDerivative operator+(const StateDerivative& other) const {
            StateDerivative result;
            result.x_dot = x_dot + other.x_dot;
            result.y_dot = y_dot + other.y_dot;
            result.z_dot = z_dot + other.z_dot;
            result.u_dot = u_dot + other.u_dot;
            result.v_dot = v_dot + other.v_dot;
            result.w_dot = w_dot + other.w_dot;
            result.phi_dot = phi_dot + other.phi_dot;
            result.theta_dot = theta_dot + other.theta_dot;
            result.psi_dot = psi_dot + other.psi_dot;
            result.p_dot = p_dot + other.p_dot;
            result.q_dot = q_dot + other.q_dot;
            result.r_dot = r_dot + other.r_dot;
            return result;
        }
    };
    
private:
    MulticopterDynamics dynamics_;
    
    // 状態に微分を加算（一時的な状態計算用）
    MulticopterState addDerivative(const MulticopterState& state, 
                                  const StateDerivative& derivative, 
                                  float dt) const {
        MulticopterState result = state;
        
        // 位置更新
        result.position.x += derivative.x_dot * dt;
        result.position.y += derivative.y_dot * dt;
        result.position.z += derivative.z_dot * dt;
        
        // 機体軸速度更新
        result.body_velocity.u += derivative.u_dot * dt;
        result.body_velocity.v += derivative.v_dot * dt;
        result.body_velocity.w += derivative.w_dot * dt;
        
        // 姿勢更新
        result.attitude.roll += derivative.phi_dot * dt;
        result.attitude.pitch += derivative.theta_dot * dt;
        result.attitude.yaw += derivative.psi_dot * dt;
        
        // 角速度更新
        result.body_angular_velocity.p += derivative.p_dot * dt;
        result.body_angular_velocity.q += derivative.q_dot * dt;
        result.body_angular_velocity.r += derivative.r_dot * dt;
        
        return result;
    }
    
public:
    // コンストラクタ
    RungeKutta4Integrator() = default;
    
    // メインの積分関数
    void integrate(MulticopterState& state, const ControlInput& input, float dt) {
        // k1の計算
        StateDerivative k1 = computeStateDerivative(state, input);
        
        // k2の計算
        MulticopterState temp_state = addDerivative(state, k1, dt * 0.5f);
        StateDerivative k2 = computeStateDerivative(temp_state, input);
        
        // k3の計算
        temp_state = addDerivative(state, k2, dt * 0.5f);
        StateDerivative k3 = computeStateDerivative(temp_state, input);
        
        // k4の計算
        temp_state = addDerivative(state, k3, dt);
        StateDerivative k4 = computeStateDerivative(temp_state, input);
        
        // RK4による状態更新
        updateState(state, k1, k2, k3, k4, dt);
    }
    
private:
    // 状態微分の計算
    StateDerivative computeStateDerivative(const MulticopterState& state, 
                                          const ControlInput& input) {
        auto derivative_data = dynamics_.computeDerivatives(state, input);
        
        StateDerivative result;
        result.x_dot = derivative_data.position_dot.x;
        result.y_dot = derivative_data.position_dot.y;
        result.z_dot = derivative_data.position_dot.z;
        result.u_dot = derivative_data.body_acceleration.u;
        result.v_dot = derivative_data.body_acceleration.v;
        result.w_dot = derivative_data.body_acceleration.w;
        result.phi_dot = derivative_data.attitude_dot.roll;
        result.theta_dot = derivative_data.attitude_dot.pitch;
        result.psi_dot = derivative_data.attitude_dot.yaw;
        result.p_dot = derivative_data.body_angular_acceleration.p;
        result.q_dot = derivative_data.body_angular_acceleration.q;
        result.r_dot = derivative_data.body_angular_acceleration.r;
        
        return result;
    }
    
    // RK4による状態更新
    void updateState(MulticopterState& state, 
                    const StateDerivative& k1, const StateDerivative& k2, 
                    const StateDerivative& k3, const StateDerivative& k4, 
                    float dt) {
        // RK4の重み付け平均: (k1 + 2*k2 + 2*k3 + k4) / 6
        StateDerivative weighted_avg = (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (1.0f / 6.0f);
        
        // 位置更新
        state.position.x += weighted_avg.x_dot * dt;
        state.position.y += weighted_avg.y_dot * dt;
        state.position.z += weighted_avg.z_dot * dt;
        
        // 機体軸速度更新
        state.body_velocity.u += weighted_avg.u_dot * dt;
        state.body_velocity.v += weighted_avg.v_dot * dt;
        state.body_velocity.w += weighted_avg.w_dot * dt;
        
        // 姿勢更新
        state.attitude.roll += weighted_avg.phi_dot * dt;
        state.attitude.pitch += weighted_avg.theta_dot * dt;
        state.attitude.yaw += weighted_avg.psi_dot * dt;
        
        // 角速度更新
        state.body_angular_velocity.p += weighted_avg.p_dot * dt;
        state.body_angular_velocity.q += weighted_avg.q_dot * dt;
        state.body_angular_velocity.r += weighted_avg.r_dot * dt;
        
        // 角度の正規化（-π～π）
        normalizeAngles(state);
    }
    
    // 角度の正規化
    void normalizeAngles(MulticopterState& state) {
        state.attitude.roll = normalizeAngle(state.attitude.roll);
        state.attitude.pitch = normalizeAngle(state.attitude.pitch);
        state.attitude.yaw = normalizeAngle(state.attitude.yaw);
    }
    
    float normalizeAngle(float angle) {
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        return angle;
    }
};
```

### 3.6.3 シミュレーション実行クラス

完全なマルチコプタシミュレーションを管理するクラス：

```cpp
// マルチコプタシミュレーション管理クラス
class MulticopterSimulation {
public:
    struct SimulationConfig {
        float dt = 0.001f;              // 積分ステップサイズ [s]
        float simulation_time = 10.0f;  // 総シミュレーション時間 [s]
        bool enable_wind = false;       // 風外乱の有効/無効
        bool enable_sensor_noise = false; // センサノイズの有効/無効
        
        SimulationConfig() = default;
    };
    
    struct SimulationResults {
        std::vector<float> time_history;
        std::vector<MulticopterState> state_history;
        std::vector<ControlInput> input_history;
        
        void clear() {
            time_history.clear();
            state_history.clear();
            input_history.clear();
        }
        
        void reserve(size_t size) {
            time_history.reserve(size);
            state_history.reserve(size);
            input_history.reserve(size);
        }
    };
    
private:
    MulticopterState current_state_;
    RungeKutta4Integrator integrator_;
    SimulationConfig config_;
    SimulationResults results_;
    float current_time_;
    
public:
    // コンストラクタ
    explicit MulticopterSimulation(const SimulationConfig& config = SimulationConfig())
        : config_(config), current_time_(0.0f) {
        resetSimulation();
    }
    
    // シミュレーション初期化
    void resetSimulation() {
        current_state_.reset();
        current_time_ = 0.0f;
        results_.clear();
        
        // 結果配列のメモリ予約
        size_t expected_steps = static_cast<size_t>(config_.simulation_time / config_.dt) + 1;
        results_.reserve(expected_steps);
    }
    
    // 初期状態の設定
    void setInitialState(const MulticopterState& initial_state) {
        current_state_ = initial_state;
    }
    
    // 1ステップ実行
    void stepSimulation(const ControlInput& input) {
        // 現在の状態を記録
        results_.time_history.push_back(current_time_);
        results_.state_history.push_back(current_state_);
        results_.input_history.push_back(input);
        
        // 積分実行
        integrator_.integrate(current_state_, input, config_.dt);
        
        // 時刻更新
        current_time_ += config_.dt;
    }
    
    // 自動シミュレーション実行（固定入力）
    void runSimulation(const ControlInput& constant_input) {
        resetSimulation();
        
        while (current_time_ < config_.simulation_time) {
            stepSimulation(constant_input);
        }
    }
    
    // カスタム制御則でのシミュレーション実行
    template<typename ControlFunction>
    void runSimulationWithControl(ControlFunction control_func) {
        resetSimulation();
        
        while (current_time_ < config_.simulation_time) {
            ControlInput input = control_func(current_state_, current_time_);
            stepSimulation(input);
        }
    }
    
    // ホバリングシミュレーション
    void runHoveringSimulation(float target_altitude = 1.0f) {
        // 重力補償のための推力計算
        float hover_thrust = MulticopterDynamics::Parameters{}.mass * 9.81f;
        
        ControlInput hover_input;
        hover_input.motor_commands[0] = hover_thrust / 4.0f;
        hover_input.motor_commands[1] = hover_thrust / 4.0f;
        hover_input.motor_commands[2] = hover_thrust / 4.0f;
        hover_input.motor_commands[3] = hover_thrust / 4.0f;
        
        // 初期高度設定
        current_state_.position.z = -target_altitude; // NED座標系（下向き正）
        
        runSimulation(hover_input);
    }
    
    // ステップ応答シミュレーション
    void runStepResponseSimulation(float step_amplitude = 0.1f, float step_time = 2.0f) {
        auto step_control = [step_amplitude, step_time](const MulticopterState& state, float time) -> ControlInput {
            float hover_thrust = MulticopterDynamics::Parameters{}.mass * 9.81f;
            ControlInput input;
            
            // ベースのホバリング推力
            float base_thrust = hover_thrust / 4.0f;
            input.motor_commands[0] = base_thrust;
            input.motor_commands[1] = base_thrust;
            input.motor_commands[2] = base_thrust;
            input.motor_commands[3] = base_thrust;
            
            // ステップ時刻以降にピッチ入力を加える
            if (time >= step_time) {
                float pitch_moment = step_amplitude;
                float arm_length = MulticopterDynamics::Parameters{}.arm_length;
                float thrust_diff = pitch_moment / arm_length;
                
                input.motor_commands[0] += thrust_diff;  // 前
                input.motor_commands[1] += thrust_diff;  // 前
                input.motor_commands[2] -= thrust_diff;  // 後
                input.motor_commands[3] -= thrust_diff;  // 後
            }
            
            return input;
        };
        
        runSimulationWithControl(step_control);
    }
    
    // 結果の取得
    const SimulationResults& getResults() const {
        return results_;
    }
    
    // 現在の状態取得
    const MulticopterState& getCurrentState() const {
        return current_state_;
    }
    
    // 現在時刻取得
    float getCurrentTime() const {
        return current_time_;
    }
    
    // 設定変更
    void setConfig(const SimulationConfig& config) {
        config_ = config;
    }
    
    // シミュレーション完了判定
    bool isSimulationComplete() const {
        return current_time_ >= config_.simulation_time;
    }
    
    // 結果の簡易表示（デバッグ用）
    void printSummary() const {
        if (results_.time_history.empty()) {
            printf("No simulation data available.\n");
            return;
        }
        
        const auto& final_state = results_.state_history.back();
        printf("Simulation Summary:\n");
        printf("  Duration: %.3f seconds\n", results_.time_history.back());
        printf("  Steps: %zu\n", results_.time_history.size());
        printf("  Final Position: (%.3f, %.3f, %.3f)\n", 
               final_state.position.x, final_state.position.y, final_state.position.z);
        printf("  Final Attitude: (%.3f, %.3f, %.3f) degrees\n", 
               final_state.attitude.roll * 180.0f / M_PI,
               final_state.attitude.pitch * 180.0f / M_PI,
               final_state.attitude.yaw * 180.0f / M_PI);
    }
};
```

## 3.7 線形化モデル

制御系設計のため、平衡点周りで線形化：

### 3.7.1 ホバリング平衡点クラス

```cpp
// ホバリング平衡点の定義と管理
class EquilibriumPoint {
public:
    struct HoverConditions {
        float altitude = 0.0f;           // ホバリング高度 [m]
        float mass = 0.0368f;           // 機体質量 [kg]
        float gravity = 9.81f;          // 重力加速度 [m/s²]
        
        HoverConditions() = default;
        HoverConditions(float alt, float m) : altitude(alt), mass(m) {}
    };
    
private:
    HoverConditions conditions_;
    MulticopterState equilibrium_state_;
    ControlInput equilibrium_input_;
    
public:
    // コンストラクタ
    explicit EquilibriumPoint(const HoverConditions& conditions = HoverConditions())
        : conditions_(conditions) {
        computeEquilibrium();
    }
    
    // 平衡点の計算
    void computeEquilibrium() {
        // 平衡状態（すべてゼロ、高度のみ指定値）
        equilibrium_state_.reset();
        equilibrium_state_.position.z = -conditions_.altitude; // NED座標系
        
        // 平衡入力（重力補償）
        float total_thrust = conditions_.mass * conditions_.gravity;
        float motor_thrust = total_thrust / 4.0f;
        
        equilibrium_input_.motor_commands[0] = motor_thrust;
        equilibrium_input_.motor_commands[1] = motor_thrust;
        equilibrium_input_.motor_commands[2] = motor_thrust;
        equilibrium_input_.motor_commands[3] = motor_thrust;
    }
    
    // 平衡状態の取得
    const MulticopterState& getState() const {
        return equilibrium_state_;
    }
    
    // 平衡入力の取得
    const ControlInput& getInput() const {
        return equilibrium_input_;
    }
    
    // 条件の変更
    void setConditions(const HoverConditions& conditions) {
        conditions_ = conditions;
        computeEquilibrium();
    }
    
    // 必要推力の計算
    float getRequiredThrust() const {
        return conditions_.mass * conditions_.gravity;
    }
    
    // モータあたりの推力
    float getMotorThrust() const {
        return getRequiredThrust() / 4.0f;
    }
};
```

### 3.7.2 線形化システム行列クラス

ホバリング平衡点周りでの線形化を管理するクラス：

```cpp
// 線形化システム行列の計算と管理
class LinearizedSystemMatrices {
public:
    static constexpr int STATE_SIZE = 12;
    static constexpr int INPUT_SIZE = 4;
    
    // 状態ベクトルのインデックス定義
    enum StateIndex {
        X_POS = 0, Y_POS = 1, Z_POS = 2,           // 位置
        U_VEL = 3, V_VEL = 4, W_VEL = 5,           // 機体軸速度
        ROLL = 6, PITCH = 7, YAW = 8,              // 姿勢
        P_RATE = 9, Q_RATE = 10, R_RATE = 11       // 角速度
    };
    
    // 入力ベクトルのインデックス定義
    enum InputIndex {
        THRUST = 0,                                 // 総推力
        ROLL_MOMENT = 1,                           // ロールモーメント
        PITCH_MOMENT = 2,                          // ピッチモーメント
        YAW_MOMENT = 3                             // ヨーモーメント
    };
    
private:
    float A_[STATE_SIZE][STATE_SIZE];              // 状態行列
    float B_[STATE_SIZE][INPUT_SIZE];              // 入力行列
    MulticopterDynamics::Parameters params_;
    
public:
    // コンストラクタ
    LinearizedSystemMatrices(const MulticopterDynamics::Parameters& params = MulticopterDynamics::Parameters())
        : params_(params) {
        computeLinearizedMatrices();
    }
    
    // 線形化行列の計算
    void computeLinearizedMatrices() {
        // 行列初期化
        memset(A_, 0, sizeof(A_));
        memset(B_, 0, sizeof(B_));
        
        // A行列の計算（機体軸座標系ベース）
        
        // 位置-速度の関係（ホバリング時は機体軸速度がそのまま地球座標系の速度微分）
        A_[X_POS][U_VEL] = 1.0f;   // dx/dt = u（前進速度）
        A_[Y_POS][V_VEL] = 1.0f;   // dy/dt = v（横方向速度）
        A_[Z_POS][W_VEL] = 1.0f;   // dz/dt = w（下方向速度）
        
        // 機体軸速度と姿勢の関係（重力の影響）
        A_[U_VEL][PITCH] = -params_.gravity;   // du/dt ≈ -g*theta（ピッチ角で前進）
        A_[V_VEL][ROLL] = params_.gravity;     // dv/dt ≈ g*phi（ロール角で横移動）
        // W軸の重力影響は平衡点で相殺されるため線形化では現れない
        
        // 姿勢-角速度の関係
        A_[ROLL][P_RATE] = 1.0f;   // dphi/dt = p
        A_[PITCH][Q_RATE] = 1.0f;  // dtheta/dt = q
        A_[YAW][R_RATE] = 1.0f;    // dpsi/dt = r
        
        // B行列の計算
        
        // 推力の影響（機体軸W方向、上向きを負とするNED座標系）
        B_[W_VEL][THRUST] = -1.0f / params_.mass;
        
        // モーメントの影響
        B_[P_RATE][ROLL_MOMENT] = 1.0f / params_.Ixx;    // ロール角速度
        B_[Q_RATE][PITCH_MOMENT] = 1.0f / params_.Iyy;   // ピッチ角速度
        B_[R_RATE][YAW_MOMENT] = 1.0f / params_.Izz;     // ヨー角速度
    }
    
    // A行列の取得
    void getAMatrix(float A[STATE_SIZE][STATE_SIZE]) const {
        memcpy(A, A_, sizeof(A_));
    }
    
    // B行列の取得
    void getBMatrix(float B[STATE_SIZE][INPUT_SIZE]) const {
        memcpy(B, B_, sizeof(B_));
    }
    
    // 特定要素のアクセス
    float getA(int i, int j) const {
        if (i >= 0 && i < STATE_SIZE && j >= 0 && j < STATE_SIZE) {
            return A_[i][j];
        }
        return 0.0f;
    }
    
    float getB(int i, int j) const {
        if (i >= 0 && i < STATE_SIZE && j >= 0 && j < INPUT_SIZE) {
            return B_[i][j];
        }
        return 0.0f;
    }
    
    // 線形化の妥当性チェック
    bool validateLinearization(const MulticopterState& state) const {
        // 小角度近似の妥当性チェック（±15度程度）
        const float max_angle = 15.0f * M_PI / 180.0f;
        
        return (abs(state.attitude.roll) < max_angle &&
                abs(state.attitude.pitch) < max_angle &&
                abs(state.body_velocity.u) < 2.0f &&  // 低速度
                abs(state.body_velocity.v) < 2.0f &&
                abs(state.body_angular_velocity.p) < 1.0f &&  // 低角速度
                abs(state.body_angular_velocity.q) < 1.0f);
    }
    
    // パラメータ更新
    void updateParameters(const MulticopterDynamics::Parameters& params) {
        params_ = params;
        computeLinearizedMatrices();
    }
    
    // 行列表示（デバッグ用）
    void printMatrices() const {
        printf("A Matrix (State Transition):\n");
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < STATE_SIZE; ++j) {
                printf("%8.4f ", A_[i][j]);
            }
            printf("\n");
        }
        
        printf("\nB Matrix (Input):\n");
        for (int i = 0; i < STATE_SIZE; ++i) {
            for (int j = 0; j < INPUT_SIZE; ++j) {
                printf("%8.4f ", B_[i][j]);
            }
            printf("\n");
        }
    }
};
```

## 3.8 外乱と摂動モデル

### 3.8.1 風外乱モデルクラス

風外乱と乱流をモデル化するクラス：

```cpp
// 風外乱モデルクラス
class WindDisturbanceModel {
public:
    struct WindParameters {
        float steady_wind_x = 0.0f;       // 定常風 X方向 [m/s]
        float steady_wind_y = 0.0f;       // 定常風 Y方向 [m/s]
        float steady_wind_z = 0.0f;       // 定常風 Z方向 [m/s]
        float turbulence_intensity = 0.1f; // 乱流強度 [-]
        float turbulence_scale = 200.0f;   // 乱流スケール長 [m]
        
        WindParameters() = default;
    };
    
    struct WindForces {
        float fx_wind, fy_wind, fz_wind;  // 風での力 [N]
        float mx_wind, my_wind, mz_wind;  // 風でのモーメント [N・m]
        
        WindForces() : fx_wind(0), fy_wind(0), fz_wind(0), 
                      mx_wind(0), my_wind(0), mz_wind(0) {}
    };
    
private:
    WindParameters params_;
    
    // 乱流状態（Drydenモデル）
    float turbulence_state_x_;
    float turbulence_state_y_;
    float turbulence_state_z_;
    
    // ランダム数生成器
    std::mt19937 rng_;
    std::normal_distribution<float> normal_dist_;
    
public:
    // コンストラクタ
    explicit WindDisturbanceModel(const WindParameters& params = WindParameters())
        : params_(params), turbulence_state_x_(0), turbulence_state_y_(0), 
          turbulence_state_z_(0), rng_(std::random_device{}()), normal_dist_(0.0f, 1.0f) {}
    
    // 乱流生成（Drydenモデル）
    void generateTurbulence(float velocity, float altitude, float dt,
                           float& vx_turb, float& vy_turb, float& vz_turb) {
        // 乱流パラメータ
        float sigma_u = params_.turbulence_intensity * std::max(velocity, 1.0f);
        float alpha = velocity / params_.turbulence_scale;
        
        // 白色ノイズ生成
        float white_noise_x = normal_dist_(rng_);
        float white_noise_y = normal_dist_(rng_);
        float white_noise_z = normal_dist_(rng_);
        
        // 1次フィルタ更新（オイラー法）
        turbulence_state_x_ += (-alpha * turbulence_state_x_ + sigma_u * white_noise_x) * dt;
        turbulence_state_y_ += (-alpha * turbulence_state_y_ + sigma_u * white_noise_y) * dt;
        turbulence_state_z_ += (-alpha * turbulence_state_z_ + sigma_u * white_noise_z) * dt;
        
        vx_turb = turbulence_state_x_;
        vy_turb = turbulence_state_y_;
        vz_turb = turbulence_state_z_;
    }
    
    // 風力の計算
    WindForces computeWindForces(const MulticopterState& state, float dt) {
        WindForces forces;
        
        // 機体軸速度から速度ノルムを計算
        float velocity = std::sqrt(state.body_velocity.u * state.body_velocity.u +
                                  state.body_velocity.v * state.body_velocity.v +
                                  state.body_velocity.w * state.body_velocity.w);
        
        // 乱流成分の生成
        float vx_turb, vy_turb, vz_turb;
        generateTurbulence(velocity, -state.position.z, dt, vx_turb, vy_turb, vz_turb);
        
        // 総風速（定常風 + 乱流）
        float wind_x = params_.steady_wind_x + vx_turb;
        float wind_y = params_.steady_wind_y + vy_turb;
        float wind_z = params_.steady_wind_z + vz_turb;
        
        // 相対風速（機体軸座標系）
        float relative_wind_x = wind_x - state.body_velocity.u;
        float relative_wind_y = wind_y - state.body_velocity.v;
        float relative_wind_z = wind_z - state.body_velocity.w;
        
        // 簡略化された風力モデル（空気抗力）
        const float drag_coefficient = 0.1f;  // 空気抗力係数
        
        forces.fx_wind = -drag_coefficient * relative_wind_x * std::abs(relative_wind_x);
        forces.fy_wind = -drag_coefficient * relative_wind_y * std::abs(relative_wind_y);
        forces.fz_wind = -drag_coefficient * relative_wind_z * std::abs(relative_wind_z);
        
        // モーメント（簡略化）
        const float moment_coefficient = 0.01f;
        forces.mx_wind = moment_coefficient * relative_wind_y;
        forces.my_wind = moment_coefficient * relative_wind_x;
        forces.mz_wind = 0.0f;  // ヨーモーメントは小さいと仮定
        
        return forces;
    }
    
    // パラメータ設定
    void setParameters(const WindParameters& params) {
        params_ = params;
    }
    
    // 乱流状態リセット
    void resetTurbulence() {
        turbulence_state_x_ = 0.0f;
        turbulence_state_y_ = 0.0f;
        turbulence_state_z_ = 0.0f;
    }
};
```

### 3.8.2 IMUセンサモデルクラス

センサノイズとバイアスをモデル化するクラス：

```cpp
// IMUセンサモデルクラス
class IMUSensorModel {
public:
    struct IMUParameters {
        // バイアス（測定誤差）
        float accel_bias_x = 0.01f;      // m/s²
        float accel_bias_y = -0.02f;
        float accel_bias_z = 0.015f;
        float gyro_bias_x = 0.001f;      // rad/s
        float gyro_bias_y = -0.002f;
        float gyro_bias_z = 0.0015f;
        
        // ノイズ標準偏差
        float accel_noise_std = 0.05f;   // m/s²
        float gyro_noise_std = 0.01f;    // rad/s
        
        // スケールファクタ（感度誤差）
        float accel_scale_factor = 1.0f; // [-]
        float gyro_scale_factor = 1.0f;  // [-]
        
        IMUParameters() = default;
    };
    
    struct IMUMeasurement {
        float accel_x, accel_y, accel_z;  // 加速度計測定値 [m/s²]
        float gyro_x, gyro_y, gyro_z;     // ジャイロ測定値 [rad/s]
        
        IMUMeasurement() : accel_x(0), accel_y(0), accel_z(0),
                          gyro_x(0), gyro_y(0), gyro_z(0) {}
    };
    
private:
    IMUParameters params_;
    RotationMatrix rotation_matrix_;
    
    // ランダム数生成器
    std::mt19937 rng_;
    std::normal_distribution<float> normal_dist_;
    
public:
    // コンストラクタ
    explicit IMUSensorModel(const IMUParameters& params = IMUParameters())
        : params_(params), rng_(std::random_device{}()), normal_dist_(0.0f, 1.0f) {}
    
    // IMU測定値のシミュレーション
    IMUMeasurement measureIMU(const MulticopterState& true_state, 
                             const BodyAcceleration& true_acceleration) {
        IMUMeasurement measurement;
        
        // 回転行列の計算（機体軸座標系から地球座標系へ）
        rotation_matrix_.updateFromEuler(true_state.attitude.roll, 
                                       true_state.attitude.pitch, 
                                       true_state.attitude.yaw);
        
        // 重力を機体軸座標系で表現
        Vector3 gravity_earth(0.0f, 0.0f, 9.81f);  // 地球座標系での重力
        Vector3 gravity_body = rotation_matrix_.transposeMultiply(gravity_earth);
        
        // 加速度計測定値（真の加速度 + 重力 + バイアス + ノイズ）
        measurement.accel_x = (true_acceleration.u + gravity_body.x) * params_.accel_scale_factor +
                             params_.accel_bias_x + params_.accel_noise_std * normal_dist_(rng_);
        measurement.accel_y = (true_acceleration.v + gravity_body.y) * params_.accel_scale_factor +
                             params_.accel_bias_y + params_.accel_noise_std * normal_dist_(rng_);
        measurement.accel_z = (true_acceleration.w + gravity_body.z) * params_.accel_scale_factor +
                             params_.accel_bias_z + params_.accel_noise_std * normal_dist_(rng_);
        
        // ジャイロ測定値（真の角速度 + バイアス + ノイズ）
        measurement.gyro_x = true_state.body_angular_velocity.p * params_.gyro_scale_factor +
                            params_.gyro_bias_x + params_.gyro_noise_std * normal_dist_(rng_);
        measurement.gyro_y = true_state.body_angular_velocity.q * params_.gyro_scale_factor +
                            params_.gyro_bias_y + params_.gyro_noise_std * normal_dist_(rng_);
        measurement.gyro_z = true_state.body_angular_velocity.r * params_.gyro_scale_factor +
                            params_.gyro_bias_z + params_.gyro_noise_std * normal_dist_(rng_);
        
        return measurement;
    }
    
    // パラメータ設定
    void setParameters(const IMUParameters& params) {
        params_ = params;
    }
    
    // バイアスキャリブレーションシミュレーション
    void calibrateBias(int num_samples = 1000) {
        // 静止状態でのバイアス計測シミュレーション
        float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
        float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
        
        MulticopterState static_state;
        static_state.reset();
        BodyAcceleration zero_accel;
        
        for (int i = 0; i < num_samples; ++i) {
            IMUMeasurement meas = measureIMU(static_state, zero_accel);
            accel_sum_x += meas.accel_x;
            accel_sum_y += meas.accel_y;
            accel_sum_z += meas.accel_z - 9.81f; // 重力を差し引いた値
            gyro_sum_x += meas.gyro_x;
            gyro_sum_y += meas.gyro_y;
            gyro_sum_z += meas.gyro_z;
        }
        
        // 平均値をバイアスとして設定
        params_.accel_bias_x = accel_sum_x / num_samples;
        params_.accel_bias_y = accel_sum_y / num_samples;
        params_.accel_bias_z = accel_sum_z / num_samples;
        params_.gyro_bias_x = gyro_sum_x / num_samples;
        params_.gyro_bias_y = gyro_sum_y / num_samples;
        params_.gyro_bias_z = gyro_sum_z / num_samples;
    }
};
```

## 3.9 数値計算最適化クラス

### 3.9.1 計算効率最適化クラス

高速な数値計算のための最適化クラス：

```cpp
// 数値計算最適化クラス
class NumericalOptimizer {
public:
    // 三角関数キャッシュクラス
    class TrigonometricCache {
    private:
        float sin_phi_, cos_phi_;
        float sin_theta_, cos_theta_;
        float sin_psi_, cos_psi_;
        bool is_valid_;
        
    public:
        TrigonometricCache() : is_valid_(false) {}
        
        void update(float phi, float theta, float psi) {
            sin_phi_ = std::sin(phi);     cos_phi_ = std::cos(phi);
            sin_theta_ = std::sin(theta); cos_theta_ = std::cos(theta);
            sin_psi_ = std::sin(psi);     cos_psi_ = std::cos(psi);
            is_valid_ = true;
        }
        
        // アクセサメソッド
        float sin_phi() const { return sin_phi_; }
        float cos_phi() const { return cos_phi_; }
        float sin_theta() const { return sin_theta_; }
        float cos_theta() const { return cos_theta_; }
        float sin_psi() const { return sin_psi_; }
        float cos_psi() const { return cos_psi_; }
        bool isValid() const { return is_valid_; }
    };
    
    // 数値安定性ユーティリティ
    class NumericalStability {
    public:
        // 安全な平方根
        static float safeSqrt(float x) {
            return (x < 0.0f) ? 0.0f : std::sqrt(x);
        }
        
        // 安全な逆三角関数
        static float safeAsin(float x) {
            if (x < -1.0f) return -M_PI / 2.0f;
            if (x > 1.0f) return M_PI / 2.0f;
            return std::asin(x);
        }
        
        static float safeAcos(float x) {
            if (x < -1.0f) return M_PI;
            if (x > 1.0f) return 0.0f;
            return std::acos(x);
        }
        
        // 除算の安全性チェック
        static float safeDivide(float numerator, float denominator, float fallback = 0.0f) {
            if (std::abs(denominator) < 1e-10f) {
                return fallback;
            }
            return numerator / denominator;
        }
        
        // 数値の有効性チェック
        static bool isValid(float value) {
            return std::isfinite(value) && !std::isnan(value);
        }
        
        // ベクトルの正規化（安全版）
        static Vector3 safeNormalize(const Vector3& vec, const Vector3& fallback = Vector3(0, 0, 1)) {
            float norm = vec.norm();
            if (norm < 1e-10f) {
                return fallback;
            }
            return vec * (1.0f / norm);
        }
        
        // 四元数の正規化チェック
        static bool checkQuaternionNorm(const Quaternion& q, float tolerance = 0.1f) {
            float norm_squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
            return std::abs(norm_squared - 1.0f) < tolerance;
        }
    };
    
    // 高速回転行列計算クラス
    class FastRotationMatrix {
    private:
        TrigonometricCache trig_cache_;
        
    public:
        void computeFromEuler(float phi, float theta, float psi, float R[3][3]) {
            trig_cache_.update(phi, theta, psi);
            
            float cp = trig_cache_.cos_phi();
            float sp = trig_cache_.sin_phi();
            float ct = trig_cache_.cos_theta();
            float st = trig_cache_.sin_theta();
            float cy = trig_cache_.cos_psi();
            float sy = trig_cache_.sin_psi();
            
            // 回転行列の計算（キャッシュされた値使用）
            R[0][0] = ct * cy;
            R[0][1] = sp * st * cy - cp * sy;
            R[0][2] = cp * st * cy + sp * sy;
            
            R[1][0] = ct * sy;
            R[1][1] = sp * st * sy + cp * cy;
            R[1][2] = cp * st * sy - sp * cy;
            
            R[2][0] = -st;
            R[2][1] = sp * ct;
            R[2][2] = cp * ct;
        }
        
        // 転置行列の計算
        void computeTranspose(const float R[3][3], float Rt[3][3]) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    Rt[i][j] = R[j][i];
                }
            }
        }
    };
    
private:
    TrigonometricCache trig_cache_;
    FastRotationMatrix fast_rotation_;
    
public:
    // コンストラクタ
    NumericalOptimizer() = default;
    
    // 三角関数キャッシュの取得
    TrigonometricCache& getTrigCache() {
        return trig_cache_;
    }
    
    // 高速回転行列計算器の取得
    FastRotationMatrix& getFastRotation() {
        return fast_rotation_;
    }
    
    // ベンチマーク関数
    void benchmarkPerformance(int iterations = 100000) {
        auto start = std::chrono::high_resolution_clock::now();
        
        for (int i = 0; i < iterations; ++i) {
            float phi = 0.1f * i;
            float theta = 0.05f * i;
            float psi = 0.02f * i;
            
            float R[3][3];
            fast_rotation_.computeFromEuler(phi, theta, psi, R);
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        
        printf("Performance: %d rotations in %ld microseconds\n", iterations, duration.count());
        printf("Average: %.2f microseconds per rotation\n", 
               static_cast<double>(duration.count()) / iterations);
    }
};
```

### 3.9.2 統合シミュレーションシステム

すべてのコンポーネントを統合したシミュレーションシステム：

```cpp
// 統合シミュレーションシステムクラス
class IntegratedSimulationSystem {
public:
    struct SystemConfiguration {
        bool enable_wind_disturbance = false;
        bool enable_sensor_noise = false;
        bool enable_numerical_optimization = true;
        bool log_data = true;
        float simulation_dt = 0.001f;
        
        SystemConfiguration() = default;
    };
    
private:
    // コアコンポーネント
    MulticopterState current_state_;
    MulticopterDynamics dynamics_;
    RungeKutta4Integrator integrator_;
    
    // 外乱モデル
    WindDisturbanceModel wind_model_;
    IMUSensorModel imu_model_;
    
    // 最適化
    NumericalOptimizer optimizer_;
    
    // 設定
    SystemConfiguration config_;
    
    // ログデータ
    std::vector<float> time_log_;
    std::vector<MulticopterState> state_log_;
    std::vector<IMUSensorModel::IMUMeasurement> imu_log_;
    
public:
    // コンストラクタ
    explicit IntegratedSimulationSystem(const SystemConfiguration& config = SystemConfiguration())
        : config_(config) {
        resetSimulation();
    }
    
    // シミュレーション初期化
    void resetSimulation() {
        current_state_.reset();
        time_log_.clear();
        state_log_.clear();
        imu_log_.clear();
        
        if (config_.enable_wind_disturbance) {
            wind_model_.resetTurbulence();
        }
    }
    
    // メインシミュレーションループ
    void runSimulation(float duration, std::function<ControlInput(const MulticopterState&, float)> control_func) {
        resetSimulation();
        
        float current_time = 0.0f;
        
        while (current_time < duration) {
            // 制御入力の計算
            ControlInput input = control_func(current_state_, current_time);
            
            // 風外乱の適用
            if (config_.enable_wind_disturbance) {
                applyWindDisturbance();
            }
            
            // 力学積分
            integrator_.integrate(current_state_, input, config_.simulation_dt);
            
            // センサシミュレーション
            if (config_.enable_sensor_noise) {
                simulateSensors();
            }
            
            // データログ
            if (config_.log_data) {
                logCurrentState(current_time);
            }
            
            current_time += config_.simulation_dt;
        }
    }
    
private:
    void applyWindDisturbance() {
        auto wind_forces = wind_model_.computeWindForces(current_state_, config_.simulation_dt);
        
        // 風力を加速度として状態に加算
        float mass = MulticopterDynamics::Parameters{}.mass;
        current_state_.body_velocity.u += wind_forces.fx_wind / mass * config_.simulation_dt;
        current_state_.body_velocity.v += wind_forces.fy_wind / mass * config_.simulation_dt;
        current_state_.body_velocity.w += wind_forces.fz_wind / mass * config_.simulation_dt;
    }
    
    void simulateSensors() {
        auto derivative = dynamics_.computeDerivatives(current_state_, ControlInput{});
        BodyAcceleration accel{derivative.body_acceleration.u, 
                              derivative.body_acceleration.v, 
                              derivative.body_acceleration.w};
        
        auto imu_measurement = imu_model_.measureIMU(current_state_, accel);
        imu_log_.push_back(imu_measurement);
    }
    
    void logCurrentState(float time) {
        time_log_.push_back(time);
        state_log_.push_back(current_state_);
    }
    
public:
    // 結果の取得
    const std::vector<MulticopterState>& getStateHistory() const {
        return state_log_;
    }
    
    const std::vector<float>& getTimeHistory() const {
        return time_log_;
    }
    
    const std::vector<IMUSensorModel::IMUMeasurement>& getIMUHistory() const {
        return imu_log_;
    }
    
    // パフォーマンステスト
    void runPerformanceTest() {
        optimizer_.benchmarkPerformance();
    }
};
```

## まとめ

本章では、マルチコプタの剛体運動をC++オブジェクト指向設計で実装する方法を詳しく学びました：

### 主要な成果：

1. **状態管理クラス**：`MulticopterState`での6自由度運動の完全な表現
2. **動力学システム**：`MulticopterDynamics`でのニュートン・オイラー方程式の実装
3. **姿勢表現システム**：`RotationMatrix`、`Quaternion`クラスでの安全な座標変換
4. **数値積分器**：`RungeKutta4Integrator`での高精度数値解法
5. **シミュレーションシステム**：`MulticopterSimulation`での統合シミュレーション
6. **外乱モデル**：`WindDisturbanceModel`、`IMUSensorModel`での現実的な環境モデル
7. **数値最適化**：`NumericalOptimizer`での高速計算と安定性

### 設計の特徴：

- **カプセル化**：各クラスは明確な責任を持ち、適切なインターフェースで連携
- **型安全性**：コンパイル時でのエラー検出と適切な型安全性
- **再利用性**：モジュラー設計でコンポーネントの独立性と再利用性を確保
- **パフォーマンス**：リアルタイム制御に必要な計算効率を達成
- **保守性**：数値安定性とエラーハンドリングを組み込み

### 次章への橋渡し：

これらのオブジェクト指向モデルは、次章で学ぶ制御システム設計の強固な基盤を提供します：

- **制御アルゴリズム**：動力学モデルをベースとしたPID制御、状態フィードバック
- **センサフュージョン**：センサモデルを活用した状態推定
- **リアルタイム実装**：数値最適化を組み込んだ高速制御ループ
- **テストフレームワーク**：シミュレーションシステムを用いた制御性能検証

次章では、これらのクラスを基盤として、実際のStampFlyで動作する制御アルゴリズムを設計・実装していきます。

## 参考文献

### 学術文献
- Bouabdallah, S. (2007). Design and control of quadrotors with application to autonomous flying. PhD thesis, EPFL
- Mellinger, D., & Kumar, V. (2011). Minimum snap trajectory generation and control for quadrotors. ICRA
- Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems
- Mahony, R., Kumar, V., & Corke, P. (2012). Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor. IEEE Robotics & Automation Magazine

### C++オブジェクト指向設計参考資料
- Stroustrup, B. (2013). The C++ Programming Language (4th Edition). Addison-Wesley
- Meyers, S. (2014). Effective Modern C++: 42 Specific Ways to Improve Your Use of C++11 and C++14. O'Reilly Media
- Gamma, E., Helm, R., Johnson, R., & Vlissides, J. (1994). Design Patterns: Elements of Reusable Object-Oriented Software. Addison-Wesley

### 実践的資料・発表資料
- [「マルチコプタの運動と制御」基礎のきそ](https://www.docswell.com/s/Kouhei_Ito/KDVNVK-2022-06-15-193343) - 102.2K views
- [StampFlyで学ぶマルチコプタ制御](https://www.docswell.com/s/Kouhei_Ito/K38V1P-2024-02-10-094123) - 314.3K views
- [2025StampFly勉強会](https://www.docswell.com/s/Kouhei_Ito/K4VR7G-2025-03-23-104258) - 57.3K views

---

*本記事は「StampFly制御システム完全ガイド」シリーズの第3章です。*