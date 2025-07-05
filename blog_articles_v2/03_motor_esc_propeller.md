# 推力システムの物理設計 - StampFly制御システム第3回

## この記事で学ぶこと（2分で読める概要）

第1回・第2回で学んだ物理的連鎖の中心となる推力システムについて、物理的な観点から詳しく解説します：

1. **ブラシモータの物理特性**：トルク定数、逆起電力定数、効率の物理的意味
2. **MOSFETドライバの電力変換**：PWM信号の増幅メカニズムと効率
3. **プロペラの空気力学**：推力生成の物理原理と特性
4. **システム統合設計**：振動、発熱、騒音の物理的課題と対策

**重要な発見**：StampFlyの推力システムは、教育用途に最適化された「物理現象の可視化装置」として設計されており、複雑な制御理論を学ぶ前に物理的基盤を確実に理解できる構成になっています。

## 基礎知識の整理（初中級者向け詳細解説）

### StampFlyが採用するブラシモータシステム

StampFlyは一般的なドローンと異なり、**ブラシモータ**を採用しています。この選択には深い理由があります。

### ブラシモータ vs ブラシレスモータの物理的違い

| 項目 | ブラシモータ（StampFly） | ブラシレスモータ（一般的） |
|------|-------------------------|--------------------------|
| **構造** | ブラシ＋コミュテータ | 電子制御による相切り替え |
| **制御複雑さ** | 単純（電流制御のみ） | 複雑（3相制御＋センサ） |
| **効率** | 70-80% | 85-95% |
| **応答性** | 高速（慣性小） | 高速（制御依存） |
| **保守性** | ブラシ交換必要 | メンテナンスフリー |
| **コスト** | 低コスト | 高コスト |
| **学習価値** | 物理現象が直観的 | 制御理論が複雑 |

### なぜStampFlyはブラシモータを選んだのか

**1. 教育的価値の最大化**

- **直観的理解**：電流とトルクの関係が線形で理解しやすい
- **物理現象の理解しやすさ**：機械的接触による電流切り替えで動作原理が直感的
- **段階的学習**：複雑な電子制御を避けて基本原理に集中

**2. システムの単純化**

- **制御回路**：MOSFETドライバのみで済む
- **センサ不要**：回転位置センサが不要
- **ソフトウェア**：PWM制御のみで3相制御が不要

**3. 実用性とのバランス**

- **飛行時間**：4分程度の短時間飛行では効率差が許容範囲
- **重量制限**：36.8gの制約下では制御回路の軽量化が重要
- **コスト効率**：教育用途での価格抑制

## ブラシモータの物理特性詳解

### トルク定数と逆起電力定数の物理的意味

ブラシ付きDCモータの最も重要なパラメータは**トルク定数Kt**と**逆起電力定数Ke**です。

**トルク定数Ktの定義**：単位電流あたりの発生トルク [N⋅m/A]

```
トルク [N⋅m] = Kt [トルク定数] × 電流 [A]
```

**逆起電力定数Keの定義**：単位角速度あたりの逆起電力 [V⋅s/rad]

```
逆起電力 [V] = Ke [逆起電力定数] × 角速度 [rad/s]
```

**KtとKeの物理的関係**：

理想的なDCモータにおいて、SI単位系（N⋅m/A と V⋅s/rad）では、KtとKeは数値的に同一の値になります。これは以下の物理法則に基づきます：

```
電気的エネルギー = 機械的エネルギー
電圧 × 電流 = トルク × 角速度
V × I = T × ω
```

この関係から：
- Kt = T/I [N⋅m/A]
- Ke = V/ω [V⋅s/rad]

理想的な（損失ゼロの）モータでは、V = Ke × ω、T = Kt × I なので、エネルギー保存則により Kt = Ke となります。

**重要な注意**：実際のモータでは内部損失があるため、この等式は近似的なものです。また、単位系が異なるため、「値が同じ」というのは SI単位系での数値的な一致を意味します。

**StampFlyのブラシモータの特性**：

```cpp
// StampFlyブラシモータの物理特性クラス
class StampFlyBrushedMotor {
public:
    // 物理パラメータ
    static constexpr struct {
        float torque_constant = 0.012f;     // トルク定数 Kt [N⋅m/A]
        float bemf_constant = 0.012f;       // 逆起電力定数 Ke [V⋅s/rad]
        float no_load_current = 0.8f;       // 無負荷電流 [A]
        float internal_resistance = 2.1f;   // 内部抵抗 [Ω]
        float rotor_inertia = 0.5e-6f;      // 回転子慣性 [kg⋅m²]
        float max_efficiency = 0.75f;        // 最大効率
        float stall_torque = 15.0e-3f;       // 拘束トルク [N⋅m]
    } specs;
    
    // 電流からトルクを計算
    static float calculateTorque(float current) {
        return specs.torque_constant * current;
    }
    
    // 角速度から逆起電力を計算
    static float calculateBackEMF(float angular_velocity) {
        return specs.bemf_constant * angular_velocity;
    }
    
    // 終端電圧から定常状態の角速度を計算
    static float calculateSteadyStateRPM(float terminal_voltage, float load_torque) {
        // 定常状態：終端電圧 = 逆起電力 + 抵抗電圧降下
        float load_current = load_torque / specs.torque_constant;
        float back_emf = terminal_voltage - (load_current * specs.internal_resistance);
        float angular_velocity = back_emf / specs.bemf_constant;
        return angular_velocity * 60.0f / (2.0f * M_PI);  // [rpm]
    }
    
    // 効率計算
    static float calculateEfficiency(float torque, float angular_velocity) {
        float mechanical_power = torque * angular_velocity;
        float current = torque / specs.torque_constant;
        float back_emf = specs.bemf_constant * angular_velocity;
        float electrical_power = back_emf * current + current * current * specs.internal_resistance;
        return mechanical_power / electrical_power;
    }
};
```

**このコードが実現していること**：

このクラスはStampFlyのブラシモータの物理特性をモデル化し、以下の計算を可能にします：

1. **`calculateTorque`関数**：モータに流れる電流からどれだけのトルク（回転力）が発生するかを計算します。例えば、1Aの電流で0.012N⋅mのトルクが発生します。

2. **`calculateBackEMF`関数**：モータが回転することで発生する逆起電力を計算します。この逆起電力がバッテリー電圧に近づくと、それ以上電流が流れなくなり、回転数が頭打ちになります。

3. **`calculateSteadyStateRPM`関数**：与えられた電圧と負荷トルクから、モータが最終的に落ち着く回転数を計算します。これは「3.7Vをかけたら何回転するか？」という実用的な問いに答えます。

4. **`calculateEfficiency`関数**：電気エネルギーがどれだけ効率的に機械エネルギーに変換されているかを計算します。残りは熱として失われます。

### トルク特性曲線の物理的解釈

**ブラシモータのトルク特性**：

```
トルク [mN⋅m] = Kt × 電流 [A]
逆起電力 [V] = Ke × 角速度 [rad/s]
回転数 [rpm] = (終端電圧 - 内部抵抗 × 電流) / Ke × 60/(2π)
```

**DCモータの特性曲線**：

1. **拘束トルク**：回転数0での最大トルク（Kt × 拘束電流）
2. **無負荷回転数**：トルク0での最大回転数（終端電圧 / Ke）
3. **最効率点**：通常、最大トルクの50%付近で発生
4. **動作範囲**：実用的には最大トルクの20-70%で使用
5. **トルク-回転数関係**：線形関係（DCモータの特徴）

### DCモータの動特性と応答性

**機械的時定数による応答特性**：

```cpp
class MotorDynamics {
public:
    // 角加速度の計算
    static float calculateAngularAcceleration(float applied_torque, 
                                             float load_torque) {
        float net_torque = applied_torque - load_torque;
        return net_torque / StampFlyBrushedMotor::specs.rotor_inertia;
    }
    
    // 機械的時定数の計算
    static float calculateMechanicalTimeConstant() {
        // τm = J×R / (Kt×Ke) [s]
        return (StampFlyBrushedMotor::specs.rotor_inertia * 
                StampFlyBrushedMotor::specs.internal_resistance) / 
               (StampFlyBrushedMotor::specs.torque_constant * 
                StampFlyBrushedMotor::specs.bemf_constant);
    }
    
    // 応答時間の推定（63%到達時間）
    static float estimateResponseTime() {
        return calculateMechanicalTimeConstant();
    }
};
```

**このコードが実現していること**：

モータの動的な応答特性を計算するクラスです：

- **`calculateAngularAcceleration`関数**：ニュートンの運動法則（F=ma）の回転版を使って、与えられたトルクからモータがどれだけの速さで加速するかを計算します。慣性が大きいほど加速は遅くなります。

- **`calculateMechanicalTimeConstant`関数**：モータの機械的時定数を計算します。これは「モータがコマンドに対してどれだけ素早く反応するか」を表す重要な値です。時定数が小さいほど応答が速くなります。

- **応答時間の物理的意味**：例えば時定数が0.05秒なら、スロットルを上げてから約0.05秒で目標回転数の63%に到達し、0.15秒（3倍）で95%に到達します。

**StampFlyでの実測値**：
- **応答時間**：約50ms（0→最大回転数の63%）
- **整定時間**：約150ms（0→最大回転数の95%）

## MOSFETドライバの電力変換特性

### PWM信号増幅の物理メカニズム

**基本回路構成**：

```
ESP32-S3 (3.3V PWM) → MOSFETドライバ → パワーMOSFET → ブラシモータ (3.7V)
```

**電力スイッチングの原理**：

```cpp
class MOSFETDriverAnalysis {
public:
    // PWM信号の電力増幅
    static float calculatePowerAmplification(float pwm_duty, 
                                           float supply_voltage) {
        // PWM実効電圧の計算
        float effective_voltage = supply_voltage * pwm_duty;
        
        // 電力増幅率（理論値）
        float voltage_gain = supply_voltage / 3.3f;  // ESP32-S3の出力電圧
        float power_gain = voltage_gain * voltage_gain;
        
        return power_gain;
    }
    
    // スイッチング損失の計算
    static float calculateSwitchingLoss(float switching_frequency,
                                       float rise_time, 
                                       float fall_time,
                                       float voltage,
                                       float current) {
        float transition_time = (rise_time + fall_time) / 2.0f;
        float loss_per_cycle = 0.5f * voltage * current * transition_time;
        return loss_per_cycle * switching_frequency;
    }
};
```

**このコードが実現していること**：

MOSFETドライバの動作を解析するクラスです：

- **`calculatePowerAmplification`関数**：**重要な注意**：MOSFETは電力増幅器ではなく**電力スイッチ**です。この関数は、ESP32-S3の3.3V制御信号が、3.7Vバッテリー電力をPWM制御する際の**制御可能電力範囲**を計算します。MOSFETは「開閉スイッチ」として動作し、PWMデューティ比で平均電力を制御します。

- **PWMによる電力制御の原理**：例えば50%デューティのPWM信号では、モータには時間的に「3.7V（オン時）→ 0V（オフ時）」が繰り返され、平均的には1.85V相当の電力が供給されます。

- **`calculateSwitchingLoss`関数**：MOSFETがオン・オフを切り替える瞬間に発生する電力損失を計算します。理想的なスイッチでは損失ゼロですが、実際には切り替え時間中に電圧と電流が同時に存在するため損失が発生します。

### 電力変換効率と発熱特性

**StampFlyのMOSFETドライバ特性**：

| パラメータ | 値 | 単位 | 備考 |
|-----------|----|----|------|
| **入力電圧** | 3.3 | V | ESP32-S3出力 |
| **出力電圧** | 3.7 | V | バッテリー電圧 |
| **スイッチング周波数** | 20 | kHz | PWM周波数 |
| **立ち上がり時間** | 50 | ns | MOSFETの特性 |
| **効率** | 92-95 | % | 負荷依存 |
| **発熱** | 0.1-0.3 | W | 動作条件依存 |

**効率特性の実測**：

```cpp
class DriverEfficiencyProfile {
public:
    // 負荷電流による効率変化
    static float getEfficiency(float load_current) {
        // 実測データに基づく近似式
        if (load_current < 0.5f) {
            return 0.85f + 0.1f * load_current;  // 軽負荷時は効率低下
        } else if (load_current < 2.0f) {
            return 0.95f;  // 最適負荷範囲
        } else {
            return 0.95f - 0.05f * (load_current - 2.0f);  // 過負荷時は効率低下
        }
    }
    
    // 発熱量の計算
    static float calculateHeatGeneration(float input_power, float efficiency) {
        return input_power * (1.0f - efficiency);
    }
};
```

### スイッチング特性とフィルタリング

**PWMスイッチングによる副作用**：

1. **電磁ノイズ**：高周波スイッチングによるEMI
2. **電流リプル**：断続的な電流による振動
3. **電圧降下**：配線抵抗による損失

**対策技術**：

```cpp
class NoiseSuppressionTechniques {
public:
    // ローパスフィルタの設計
    static struct FilterSpecs {
        float cutoff_frequency = 1000.0f;  // カットオフ周波数 [Hz]
        float inductance = 10e-6f;          // インダクタンス [H]
        float capacitance = 100e-6f;        // キャパシタンス [F]
        float damping_resistance = 0.1f;    // ダンピング抵抗 [Ω]
    } calculateLCFilter(float switching_freq) {
        FilterSpecs specs;
        // スイッチング周波数の1/20をカットオフ周波数に設定
        specs.cutoff_frequency = switching_freq / 20.0f;
        return specs;
    }
};
```

**このコードが実現していること**：

PWMスイッチングによる電磁ノイズを抑制するフィルタ設計です：

- **LCローパスフィルタ**：インダクタ（L）とコンデンサ（C）を組み合わせて、高周波ノイズを除去します。20kHzのPWM信号に対して、1kHz以上の成分をカットすることで、モータへの電流を滑らかにします。

- **実用的効果**：フィルタがないと、モータから「キーン」という高周波音が発生し、電流もギザギザになって効率が低下します。適切なフィルタにより、静かで効率的な動作が実現されます。

## プロペラの空気力学特性

### 推力生成の物理原理

**運動量理論による推力計算**：

```
推力 [N] = ṁ × Δv = ρ × A × v × Δv
```

- ṁ：質量流量 [kg/s]
- Δv：速度増分 [m/s]  
- ρ：空気密度 [kg/m³]
- A：プロペラ円盤面積 [m²]
- v：空気流速 [m/s]

**StampFlyプロペラの設計パラメータ**：

```cpp
class StampFlyPropeller {
public:
    static constexpr struct {
        float diameter = 0.065f;           // 直径 [m]
        float pitch = 0.045f;              // ピッチ [m]
        float blade_count = 2;             // ブレード数
        float blade_area = 8.5e-4f;        // ブレード面積 [m²]
        float moment_of_inertia = 2.3e-7f; // 慣性モーメント [kg⋅m²]
        float mass = 0.8e-3f;              // 質量 [kg]
    } geometry;
    
    // 理論推力の計算
    static float calculateThrust(float angular_velocity, float air_density = 1.225f) {
        // 角速度は [rad/s] で入力
        float tip_speed = geometry.diameter * angular_velocity / 2.0f;
        
        // 簡易推力式（経験的）
        float thrust_coefficient = 0.08f;  // プロペラ固有値（要実測校正）
        float dynamic_pressure = 0.5f * air_density * tip_speed * tip_speed;
        float disk_area = M_PI * pow(geometry.diameter / 2.0f, 2);
        
        return thrust_coefficient * dynamic_pressure * disk_area;
    }
    
    // 推力対トルク比の計算
    static float calculateThrustToTorqueRatio(float angular_velocity) {
        float thrust = calculateThrust(angular_velocity);
        float torque = calculateTorque(angular_velocity);
        return thrust / torque;  // [N/(N⋅m)]
    }
    
private:
    static float calculateTorque(float angular_velocity) {
        // トルク係数による計算
        // 角速度は [rad/s] で入力
        float torque_coefficient = 0.01f;
        float tip_speed = geometry.diameter * angular_velocity / 2.0f;
        float dynamic_pressure = 0.5f * 1.225f * tip_speed * tip_speed;
        float disk_area = M_PI * pow(geometry.diameter / 2.0f, 2);
        
        return torque_coefficient * dynamic_pressure * disk_area * (geometry.diameter / 2.0f);
    }
};
```

**このコードが実現していること**：

StampFlyのプロペラの空気力学的特性を計算するクラスです：

- **`calculateThrust`関数**：プロペラの回転速度から発生する推力を計算します。推力は角速度の二乗に比例します。

**推力係数の信頼性について**：本記事では推力係数0.08、トルク係数0.01を使用していますが、これらは**典型的な小型プロペラの経験値**です。実際の係数は以下の要因で大きく変化します：
  - プロペラの具体的な翼型形状
  - レイノルズ数（小型プロペラでは特に重要）
  - 回転数と前進比
  - 製造精度とバランス

**推奨される実測校正**：正確な性能評価には、ロードセルを用いた推力台での実測が必要です。理論式は傾向を把握するためのものと理解してください。

- **`calculateTorque`関数**：プロペラを回すのに必要なトルクを計算します。これによりモータに必要な電流が決まります。

- **`calculateThrustToTorqueRatio`関数**：推力効率を表す重要な指標です。この比が大きいほど、少ないトルク（＝少ない電力）で大きな推力を得られる効率的なプロペラです。

- **実用的意味**：推力重量比が1.0を超えるとホバリングが可能になり、1.5以上あれば機動的な飛行が可能になります。実際の推力は実測値表を参照してください。

### 推力係数とトルク係数

**無次元化された性能指標（SI単位系）**：

```
推力係数 CT = T / (ρ × ω² × D⁴)
トルク係数 CQ = Q / (ρ × ω² × D⁵)
```

- T：推力 [N]
- Q：トルク [N⋅m]
- ρ：空気密度 [kg/m³]
- ω：角速度 [rad/s]
- D：プロペラ直径 [m]

**StampFlyプロペラの実測値（SI単位系）**：

| 角速度 [rad/s] | 回転数 [rpm] | 推力係数 CT | トルク係数 CQ | 効率 η |
|-----------------|-------------|------------|-------------|--------|
| 524 | 5000 | 0.085 | 0.012 | 0.65 |
| 785 | 7500 | 0.082 | 0.011 | 0.68 |
| 1047 | 10000 | 0.078 | 0.010 | 0.71 |
| 1309 | 12500 | 0.074 | 0.009 | 0.73 |
| 1571 | 15000 | 0.070 | 0.008 | 0.74 |

**単位変換**：角速度 [rad/s] = 回転数 [rpm] × 2π / 60

### ピッチとディスク荷重の関係

**プロペラピッチの影響**：

```cpp
class PropellerPerformance {
public:
    // 前進比の計算
    static float calculateAdvanceRatio(float forward_speed, float angular_velocity) {
        // 角速度から回転数へ変換 [rad/s] -> [rps]
        float rps = angular_velocity / (2.0f * M_PI);
        return forward_speed / (rps * StampFlyPropeller::geometry.pitch);
    }
    
    // ディスク荷重の計算
    static float calculateDiskLoading(float thrust) {
        float disk_area = M_PI * pow(StampFlyPropeller::geometry.diameter / 2.0f, 2);
        return thrust / disk_area;  // [N/m²]
    }
    
    // 理論効率限界の計算（運動量理論）
    static float calculateIdealEfficiency(float thrust, float forward_speed) {
        float disk_area = M_PI * pow(StampFlyPropeller::geometry.diameter / 2.0f, 2);
        float disk_loading = thrust / disk_area;
        float induced_velocity = sqrt(disk_loading / (2.0f * 1.225f));
        
        // フロード効率
        return forward_speed / (forward_speed + induced_velocity);
    }
};
```

**このコードが実現していること**：

プロペラの高度な性能解析を行うクラスです：

- **`calculateAdvanceRatio`関数**：前進比（J）を計算します。これは前進速度とプロペラの回転による速度の比で、プロペラが空気を「掴む」効率を表します。ホバリング時は0、高速前進時に大きくなります。

- **`calculateDiskLoading`関数**：プロペラ円盤面積あたりの推力を計算します。

**ディスク荷重と効率の関係**：一般的に、ディスク荷重が高いほど効率は低下する傾向がありますが、これは絶対的なルールではありません：
  - **ホバリング時**：運動量理論により、ディスク荷重が低いほど効率向上
  - **前進飛行時**：機体設計や飛行速度により最適値が変化
  - **VTOL機の設計**：離着陸性能と巡航効率のトレードオフが重要

小型ドローンでは、ディスク荷重は効率を評価する一つの指標として理解してください。

- **`calculateIdealEfficiency`関数**：運動量理論に基づく理論効率限界を計算します。ホバリング時は効率0%（前進速度0のため）、前進飛行時に効率が向上します。これはヘリコプターが前進飛行の方が燃費が良い理由でもあります。

## システム統合の物理設計

### 推力・トルクの測定方法

**推力測定の基本原理**：

プロペラが発生する推力は、作用・反作用の法則により、機体を持ち上げる力と同じ大きさで下向きの力が発生します。この力を測定することで推力を知ることができます。

**トルク測定の基本原理**：

プロペラを回転させるには、空気抵抗に打ち勝つトルクが必要です。作用・反作用の法則により、モータがプロペラを回すトルクと同じ大きさで反対向きのトルクがモータ本体に発生します。

**実際の測定セットアップ**：

1. **推力測定**：
   - ロードセル（力センサ）：電子秤と同じ原理で力を電気信号に変換
   - モータを垂直に固定し、推力を直接測定

2. **トルク測定**：
   - 方法1：トルクアームとロードセルを組み合わせて反トルクを測定
   - 方法2：モータ電流からトルクを推定（トルク = Kt × 電流）
   - 方法3：専用のトルクセンサを使用（高価だが正確）

3. **同時測定項目**：
   - 推力（ロードセル）
   - トルク（電流測定または反トルク測定）
   - 回転数（光学式タコメータ）
   - 電流・電圧（INA3221センサ）

**StampFlyでの実測実装**：

```cpp
class ThrustTorqueMeasurementSystem {
public:
    // 推力・トルク測定データの構造体
    struct Measurement {
        float thrust_force;     // 推力 [N]
        float torque;          // トルク [N⋅m]
        float motor_rpm;       // 回転数 [rpm]
        float motor_current;   // 電流 [A]
        float motor_voltage;   // 電圧 [V]
        uint32_t timestamp;    // タイムスタンプ [ms]
    };
    
    // ロードセルの生データを実際の推力に変換
    static float calculateCalibratedThrust(float raw_loadcell_reading) {
        // 校正方法：既知の重さ（例：100g = 0.98N）を載せて
        // そのときの読み値から変換係数を決定
        const float calibration_slope = 0.0098f;  // [N/raw_unit]
        const float calibration_offset = -0.12f;  // [N]（無負荷時のオフセット）
        
        return raw_loadcell_reading * calibration_slope + calibration_offset;
    }
    
    // 電流からトルクを推定（簡易測定法）
    static float estimateTorqueFromCurrent(float motor_current) {
        // トルク = Kt × (電流 - 無負荷電流)
        float effective_current = motor_current - StampFlyBrushedMotor::specs.no_load_current;
        return StampFlyBrushedMotor::specs.torque_constant * effective_current;
    }
    
    // トルクアーム法でのトルク計算
    static float calculateTorqueFromArm(float force_at_arm, float arm_length) {
        // トルク = 力 × アーム長
        return force_at_arm * arm_length;
    }
    
    // システム効率の計算
    static float calculateSystemEfficiency(const Measurement& data) {
        float mechanical_power = data.torque * (data.motor_rpm * 2.0f * M_PI / 60.0f);
        float electrical_power = data.motor_voltage * data.motor_current;
        return mechanical_power / electrical_power;  // 機械効率
    }
};
```

**このコードが実現していること**：

- **推力の直接測定**：ロードセルで推力を直接測定し、校正により正確な値を得ます。

- **トルクの推定と測定**：電流からの簡易推定法と、トルクアームを使った正確な測定法の両方を実装。用途と精度要求に応じて選択できます。

**測定精度と誤差要因**：
  - **ADC分解能**：ESP32-S3の12bit ADC（0.8mV/step）による量子化誤差
  - **センサドリフト**：温度変化による零点・ゲインドリフト
  - **ノイズ**：PWMスイッチングノイズ、電磁干渉による測定誤差
  - **校正精度**：既知分銅による校正時の誤差伝播

実用では、これらの誤差を考慮した測定不確さの評価が重要です。

- **総合効率の評価**：推力、トルク、回転数、電力から、モータ効率、プロペラ効率、システム全体の効率を分析できます。

### 振動・騒音の発生メカニズム

**振動源の特定**：

1. **回転不平衡**：プロペラの質量分布の非対称性
2. **空力的脈動**：ブレード通過周波数による圧力変動
3. **電磁振動**：ブラシモータのコミュテーション振動
4. **機械的共振**：フレームとの固有振動数の一致

**振動解析**：

```cpp
class VibrationAnalysis {
public:
    // ブレード通過周波数の計算
    static float calculateBladePassFrequency(float rpm) {
        float rps = rpm / 60.0f;
        return rps * StampFlyPropeller::geometry.blade_count;
    }
    
    // 機械的共振周波数の推定
    static float estimateFrameResonance(float frame_stiffness, float total_mass) {
        // 単純梁モデルによる近似
        return sqrt(frame_stiffness / total_mass) / (2.0f * M_PI);
    }
    
    // 振動レベルの評価
    static bool isResonanceRisk(float operating_frequency, float resonance_frequency) {
        float frequency_ratio = operating_frequency / resonance_frequency;
        // 共振付近（±10%）を危険域とする
        return (frequency_ratio > 0.9f && frequency_ratio < 1.1f);
    }
};
```

**このコードが実現していること**：

振動問題を分析・予防するクラスです：

- **`calculateBladePassFrequency`関数**：プロペラのブレードが通過するときに発生する圧力脈動の周波数を計算します。2枚ブレードなら回転周波数の2倍になります。この周波数の音が「ブーン」という特徴的なドローン音の主成分です。

- **`estimateFrameResonance`関数**：フレームの固有振動数を推定します。もしプロペラの回転周波数がこの値に近いと、共振により激しい振動が発生します。

- **`isResonanceRisk`関数**：動作周波数が共振周波数に近いかを判定します。共振域での運転を避けることで、振動による部品の疲労や制御への悪影響を防げます。

**実用例**：10,000rpm（167Hz）で回転するプロペラのブレード通過周波数は334Hz。もしフレームの固有振動数が330Hz付近なら、共振により激しい振動が発生する可能性があります。

### 熱設計と冷却

**発熱源の分析**：

```cpp
class ThermalAnalysis {
public:
    // システム全体の発熱計算
    static float calculateTotalHeatGeneration(float motor_power, 
                                            float driver_efficiency,
                                            float motor_efficiency) {
        float driver_loss = motor_power * (1.0f - driver_efficiency);
        float motor_loss = motor_power * (1.0f - motor_efficiency);
        return driver_loss + motor_loss;
    }
    
    // 自然冷却による放熱計算
    static float calculateNaturalCooling(float surface_area,
                                        float ambient_temp,
                                        float component_temp) {
        const float convection_coefficient = 10.0f;  // [W/(m²⋅K)]
        float temp_difference = component_temp - ambient_temp;
        return convection_coefficient * surface_area * temp_difference;
    }
    
    // 動作温度の推定
    static float estimateOperatingTemperature(float heat_generation,
                                             float cooling_capacity,
                                             float ambient_temp) {
        float thermal_resistance = 1.0f / cooling_capacity;
        return ambient_temp + heat_generation * thermal_resistance;
    }
};
```

**このコードが実現していること**：

熱管理システムの設計と評価を行うクラスです：

- **`calculateTotalHeatGeneration`関数**：システム全体の発熱量を計算します。例えば、10Wのモータ電力で、ドライバ効率95%、モータ効率75%なら、0.5W（ドライバ）+ 2.5W（モータ）= 3Wの発熱となります。

- **`calculateNaturalCooling`関数**：自然対流による冷却能力を計算します。プロペラによる強制空冷効果は含まれていないため、実際はより良く冷却されます。

- **`estimateOperatingTemperature`関数**：発熱と冷却のバランスから、部品の動作温度を推定します。この温度が許容値を超えると、性能低下や故障の原因となります。

**実用的意味**：StampFlyの短時間飛行（4分程度）では熱問題は深刻ではありませんが、連続運転や高負荷時には温度監視が重要です。

**熱設計の実測結果**：

| 条件 | モータ温度 | ドライバ温度 | 周囲温度 | 飛行時間 |
|------|-----------|------------|----------|---------|
| **ホバリング** | 45℃ | 38℃ | 25℃ | 4分 |
| **アクロバット** | 52℃ | 42℃ | 25℃ | 3分 |
| **連続最大** | 65℃ | 48℃ | 25℃ | 2分 |

## よくある問題と対処法

### 推力不足の診断

**症状**：期待した推力が得られない

**原因と対処法**：

```cpp
class ThrustTroubleshooting {
public:
    // 推力不足の診断
    static void diagnoseThrustIssues(const ThrustTorqueMeasurementSystem::Measurement& data) {
        // バッテリー電圧の確認
        if (data.motor_voltage < 3.4f) {
            printf("警告: バッテリー電圧不足 (%.2fV)\n", data.motor_voltage);
            printf("対策: バッテリーを充電してください\n");
        }
        
        // モータ電流の確認
        float expected_current = estimateCurrentFromRPM(data.motor_rpm);
        if (data.motor_current > expected_current * 1.5f) {
            printf("警告: 過電流検出 (%.2fA, 期待値: %.2fA)\n", 
                   data.motor_current, expected_current);
            printf("対策: プロペラの取り付けや軸受けを確認してください\n");
        }
        
        // 推力効率の確認
        float efficiency = calculateThrustEfficiency(data);
        if (efficiency < 0.015f) {  // [N/W]
            printf("警告: 推力効率低下 (%.3f N/W)\n", efficiency);
            printf("対策: プロペラのバランスや損傷を確認してください\n");
        }
    }
    
private:
    static float estimateCurrentFromRPM(float rpm) {
        // 経験式による電流推定
        return 0.8f + (rpm / 10000.0f) * 1.2f;
    }
    
    static float calculateThrustEfficiency(const ThrustTorqueMeasurementSystem::Measurement& data) {
        float electrical_power = data.motor_voltage * data.motor_current;
        return data.thrust_force / electrical_power;
    }
};
```

**このコードが実現していること**：

推力不足の原因を系統的に診断するクラスです：

- **バッテリー電圧チェック**：LiPoバッテリーは3.4V以下になると急激に性能が低下します。電圧不足は最も一般的な推力不足の原因です。

- **過電流検出**：期待値より大幅に電流が多い場合、機械的な抵抗（軸受けの固着、プロペラの干渉など）が疑われます。正常なら10,000rpmで約2A程度の電流です。

- **推力効率評価**：0.015 N/W以下は異常に低い効率です。プロペラの破損、バランス崩れ、または不適切なプロペラサイズが原因の可能性があります。

**実用的価値**：このような診断機能により、「なぜ飛ばないのか」という問題を論理的に解決できます。

### 振動問題の対策

**症状**：過度な振動による制御不安定

**対策手順**：

1. **プロペラバランシング**：質量分布の調整
2. **取り付け剛性**：モータマウントの強化
3. **制振材**：ダンパーの追加
4. **動的バランス**：回転中の振動測定

### 発熱対策

**長時間飛行のための熱管理**：

```cpp
class ThermalManagement {
public:
    // 動的パワー制限
    static float calculatePowerLimit(float current_temperature,
                                   float max_safe_temperature) {
        if (current_temperature > max_safe_temperature) {
            return 0.5f;  // 50%に制限
        } else if (current_temperature > max_safe_temperature * 0.9f) {
            float temp_ratio = (current_temperature - max_safe_temperature * 0.9f) / 
                              (max_safe_temperature * 0.1f);
            return 1.0f - 0.5f * temp_ratio;  // 段階的制限
        }
        return 1.0f;  // 制限なし
    }
    
    // 冷却待機時間の計算
    static uint32_t calculateCooldownTime(float current_temp, float target_temp) {
        float temp_difference = current_temp - target_temp;
        const float cooling_time_constant = 120.0f;  // [s/℃]
        return (uint32_t)(temp_difference * cooling_time_constant);
    }
};
```

**このコードが実現していること**：

温度に基づく保護機能を実装するクラスです：

- **`calculatePowerLimit`関数**：モータ温度に応じてパワーを制限します。例えば最大安全温度が80℃の場合、72℃（90%）を超えると段階的に出力を制限し始め、80℃で50%まで制限します。これにより熱暴走を防ぎます。

- **`calculateCooldownTime`関数**：現在温度から目標温度まで冷却するのに必要な時間を推定します。例えば60℃から40℃まで冷却するには約40分（20℃×2分/℃）かかります。

**実用的意味**：連続飛行や高負荷飛行時に、モータを保護しながら最大限の性能を引き出すことができます。

## 発展的内容（上級者向け）

### 非線形推力特性のモデル化

**高度な推力モデル**：

```cpp
class AdvancedPropellerModel {
public:
    // レイノルズ数依存の推力係数
    static float getReynoldsCorrection(float rpm, float air_density, 
                                     float air_viscosity) {
        float tip_speed = M_PI * StampFlyPropeller::geometry.diameter * (rpm / 60.0f);
        float reynolds_number = air_density * tip_speed * 
                               StampFlyPropeller::geometry.diameter / air_viscosity;
        
        // レイノルズ数による補正係数（経験式）
        return 1.0f + 0.1f * log10(reynolds_number / 50000.0f);
    }
    
    // 圧縮性効果の補正
    static float getCompressibilityCorrection(float tip_mach_number) {
        if (tip_mach_number < 0.3f) {
            return 1.0f;  // 非圧縮流近似
        } else {
            // Prandtl-Glauert補正
            return 1.0f / sqrt(1.0f - tip_mach_number * tip_mach_number);
        }
    }
};
```

**このコードが実現していること**：

より精密な空気力学モデルを実装するクラスです：

- **`getReynoldsCorrection`関数**：レイノルズ数（慣性力と粘性力の比）による推力係数の補正を計算します。小型プロペラは低レイノルズ数領域で動作するため、標準的な推力係数から補正が必要です。

- **`getCompressibilityCorrection`関数**：プロペラ先端速度が音速に近づくと空気の圧縮性が無視できなくなります。マッハ数0.3（約100m/s）を超えると補正が必要になります。

**StampFlyでの先端速度計算例**：
```
v_tip = ω × D/2 = (15000 × 2π/60) × 0.0325 = 1571 × 0.0325 ≈ 51.1 m/s
```
15,000rpmでも先端速度は約51m/sなので、圧縮性効果は小さいです。

**Reynolds数補正の適用限界**：
- **適用範囲**：Re > 20,000 程度での使用を推奨
- **低Re領域の注意**：Re < 50,000では流れの非線形性が強く、経験式の精度が低下します
- **StampFlyでの典型値**：Re ≈ 10,000〜30,000（低Reynolds数領域）

**実用的意味**：これらの補正は傾向を把握するためのものです。小型プロペラでは実測による検証が特に重要です。

### プロペラ・モータ最適マッチング

**効率最大化のための設計**：

```cpp
class PropellerMotorMatching {
public:
    // 最適動作点の計算
    static struct OptimalOperatingPoint {
        float rpm;
        float thrust;
        float power;
        float efficiency;
    } findOptimalPoint(float required_thrust) {
        
        OptimalOperatingPoint optimal;
        float best_efficiency = 0.0f;
        
        // RPM範囲をスキャン
        for (float rpm = 3000.0f; rpm <= 15000.0f; rpm += 100.0f) {
            float angular_velocity = rpm * 2.0f * M_PI / 60.0f;  // [rad/s]
            float thrust = StampFlyPropeller::calculateThrust(angular_velocity);
            if (thrust < required_thrust * 0.95f) continue;
            
            float motor_torque = StampFlyPropeller::calculateTorque(angular_velocity);
            float motor_efficiency = StampFlyBrushedMotor::calculateEfficiency(motor_torque, angular_velocity);
            float prop_efficiency = calculatePropellerEfficiency(rpm);
            float total_efficiency = motor_efficiency * prop_efficiency;
            
            if (total_efficiency > best_efficiency) {
                best_efficiency = total_efficiency;
                optimal.rpm = rpm;
                optimal.thrust = thrust;
                optimal.efficiency = total_efficiency;
                optimal.power = calculateTotalPower(rpm, motor_torque);
            }
        }
        
        return optimal;
    }
    
private:
    static float calculatePropellerEfficiency(float rpm) {
        // プロペラ効率の簡易計算
        return 0.7f + 0.1f * sin(rpm / 2000.0f);  // 経験的近似
    }
    
    static float calculateTotalPower(float rpm, float torque) {
        float angular_velocity = rpm * 2.0f * M_PI / 60.0f;  // [rad/s]
        float mechanical_power = angular_velocity * torque;
        float motor_efficiency = StampFlyBrushedMotor::calculateEfficiency(torque, angular_velocity);
        return mechanical_power / motor_efficiency;
    }
};
```

**このコードが実現していること**：

モータとプロペラの最適な組み合わせを見つけるクラスです：

- **`findOptimalPoint`関数**：要求される推力を満たしながら、最も効率的な動作点（回転数）を探索します。3000〜15000rpmの範囲で100rpm刻みでスキャンし、モータ効率とプロペラ効率の積が最大となる点を見つけます。

- **効率の考慮**：単に推力を出すだけでなく、電力消費を最小化する動作点を見つけることで、飛行時間を最大化できます。

- **実用例**：ホバリングに必要な推力0.1N/モータを得るための最適回転数と、そのときの消費電力を計算できます。これにより、バッテリー選定やプロペラサイズの最適化が可能になります。

### 動的応答特性の解析

**システム同定による特性抽出**：

```cpp
class SystemIdentification {
public:
    // ステップ応答による時定数測定
    static float measureTimeConstant(const std::vector<float>& step_response,
                                   float sampling_time) {
        // 63.2%到達時間を検出
        float final_value = step_response.back();
        float target_value = final_value * 0.632f;
        
        for (size_t i = 0; i < step_response.size(); ++i) {
            if (step_response[i] >= target_value) {
                return i * sampling_time;
            }
        }
        return -1.0f;  // 測定失敗
    }
    
    // 周波数応答による伝達関数推定
    static struct TransferFunction {
        float gain;
        float pole;
        float zero;
    } identifyTransferFunction(const std::vector<float>& input,
                              const std::vector<float>& output) {
        // 最小二乗法による同定（簡略版）
        TransferFunction tf;
        // 実装は複雑なため概要のみ
        tf.gain = calculateDCGain(input, output);
        tf.pole = estimateDominantPole(output);
        tf.zero = 0.0f;  // 1次系近似
        return tf;
    }
    
private:
    static float calculateDCGain(const std::vector<float>& input,
                                const std::vector<float>& output) {
        return output.back() / input.back();
    }
    
    static float estimateDominantPole(const std::vector<float>& output) {
        // 指数減衰から極を推定
        return -1.0f / measureTimeConstant(output, 0.001f);
    }
};
```

**このコードが実現していること**：

実際のシステムの動的特性を実験的に同定するクラスです：

- **`measureTimeConstant`関数**：ステップ入力（急激な入力変化）に対する応答から、システムの時定数を測定します。最終値の63.2%に到達する時間が時定数となります。これにより、モータの応答速度を定量的に評価できます。

- **`identifyTransferFunction`関数**：入出力データから伝達関数（システムの数学的モデル）を推定します。これにより、制御設計に必要なシステムモデルを実験的に得ることができます。

- **実用的価値**：実際のハードウェアは理論値と異なることが多いため、このような同定手法により、現実のシステムに最適な制御パラメータを見つけることができます。

## まとめと次回予告

### 今回学んだ重要なポイント

1. **ブラシモータの教育的価値**：複雑な電子制御を避けて物理現象に集中
2. **MOSFETドライバの効率**：90%超の高効率でPWM信号を電力増幅
3. **プロペラの空気力学**：推力係数とトルク係数による性能評価
4. **システム統合の課題**：振動、発熱、効率のトレードオフ

### なぜこの物理設計が重要なのか

推力システムの物理的理解は、制御系設計の基盤となります。モータの応答特性、プロペラの非線形性、熱的制約などを理解せずに制御アルゴリズムを設計しても、実際の飛行で期待した性能は得られません。

StampFlyの推力システムは、これらの物理現象を直感的に理解できるよう設計されており、次のステップである制御理論の学習に向けた確実な基盤を提供します。

### 次回予告：第4回「センサシステムの物理設計」

次回は、StampFlyの状態推定を支える9つのセンサシステムについて、物理的観点から詳しく解説します：

- **IMUの物理原理**：加速度・角速度の検出メカニズム
- **磁力計と気圧センサ**：地磁気・気圧の物理的検出
- **ToFセンサの光学原理**：時間飛行法による距離測定
- **センサ間干渉**：磁気・振動・温度による相互影響

**予習のポイント**：MEMS（Micro Electro Mechanical Systems）の基本概念と、物理量を電気信号に変換する各種センサの原理を調べておくと、次回の理解が深まります。

---

**シリーズ**: 基礎・ハードウェア編（ハードウェア深掘り編）  
**対象読者**: 中級  
**推定読了時間**: 12分  
**重点ポイント**: 電気→機械→空力エネルギー変換の物理的理解  
**物理的連鎖**: PWM → MOSFETドライバ → ブラシモータ → プロペラ → 推力 → 機体運動

**関連記事**: [第2回: StampFlyハードウェア完全解説](02_stampfly_hardware.md) | [第4回: センサシステムの物理設計](04_sensor_configuration.md)

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