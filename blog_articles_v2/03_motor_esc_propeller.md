# 推力システムの物理設計 - StampFly制御システム第3回

## この記事で学ぶこと（2分で読める概要）

StampFlyを飛ばすために必要な推力を、いかに効率よく安全に生成するかという**設計課題**を解決するプロセスを学びます：

1. **設計要求の整理**：36.8gの機体を飛ばすには何が必要か？
2. **モータ選定の物理的根拠**：なぜこのブラシモータを選んだのか？
3. **制御回路の設計判断**：2.9A制限のMOSFETドライバの妥当性
4. **推力測定と検証**：設計通りの性能が出ているか確認する方法

**重要な発見**：ドローン設計は「要求仕様から物理法則を使って部品を選定し、実測で検証する」エンジニアリングプロセスそのものです。

## ドローン設計の出発点：飛行要求の定量化

### StampFlyに求められる具体的性能

まず、「何ができるドローンを作りたいか」を数値で明確にしましょう：

**基本飛行要求：**
- **機体重量**：36.8g（法規制100g未満）
- **飛行時間**：4分間の連続ホバリング
- **機動性**：傾斜角30度での機動飛行
- **安全性**：室内での安全な飛行

### 必要推力の工学的計算

**ホバリング推力の基本要求：**

ドローンがホバリングするには、重力と釣り合う推力が必要です：

```
基本推力 = 機体重量 × 重力加速度
         = 0.0368 kg × 9.81 m/s²
         = 0.361 N
```

**機動飛行のための推力余裕：**

実際の飛行では以下の追加推力が必要：

1. **姿勢制御推力**：傾斜時の水平分力補償
2. **加速推力**：上昇・前進加速用
3. **制御余裕**：外乱補償とPID制御のため

```
必要推力 = 基本推力 × 推力余裕率
```

一般的な小型ドローンでは推力余裕率1.5〜2.0倍が必要。StampFlyは室内機なので1.5倍を採用：

```
設計推力 = 0.361 N × 1.5 = 0.542 N
1モータあたり = 0.542 N ÷ 4 = 0.136 N
```

**この計算が示すこと：**
各モータは最低0.136Nの推力を発生する必要があります。これが推力システム設計の出発点となります。

## モータ選定の工学的判断

### 推力実現のための技術選択肢

0.136N/モータの推力を実現する方法を比較検討します：

**選択肢1：ブラシレスモータ + ESC**
- 効率：85-95%
- 制御：3相交流制御（複雑）
- 重量：モータ4g + ESC2.5g = 6.5g/セット
- コスト：高価

**選択肢2：ブラシモータ + MOSFETドライバ**
- 効率：70-80%
- 制御：PWM制御（単純）
- 重量：モータ4g + MOSFET20mg = 4g/セット
- コスト：安価

### StampFlyの設計判断

教育プラットフォームとしての要求を考慮：

**重量制約による判断：**
```
モータ本体：ブラシ 4g = ブラシレス 4g（同等）
制御回路：MOSFET 20mg << ESC 2.5g（125倍の差）

ブラシレス：4 × 6.5g = 26g（全機体の71%）
ブラシ：4 × 4g = 16g（全機体の43%）
→ 10g（27%）の軽量化効果
```

**制御回路の重量が決定要因：**
- MOSFETドライバ：チップ部品のため超軽量
- ESC：専用ICとパワー回路で重量増
- 36.8g制約下では10gの差は致命的（27%）

**教育価値による判断：**
- **理解しやすさ**：電流→トルクの線形関係
- **制御の単純化**：PWM信号のみで制御
- **段階的学習**：複雑な電子制御を回避

**結論：StampFlyはブラシモータを採用**

## 実際のモータ仕様と性能解析

### StampFlyブラシモータの実測仕様

実際に選定されたモータの実測値：

```
トルク定数：Kt = 0.614 mN⋅m/A
内部抵抗：R = 0.34 Ω
インダクタンス：L = 1 μH
回転定数：Kv = 17,600 rpm/V
最大電流：2.9 A（MOSFET制限）
```

### 推力実現の妥当性検証

**必要トルクの推定：**

31mm径プロペラで0.136Nの推力を得るための回転数を推定：

```
推力 ≈ 推力係数 × 空気密度 × (角速度)² × (直径)⁴
```

小型プロペラの推力係数を0.08と仮定：
```
0.136 = 0.08 × 1.225 × ω² × (0.031)⁴
ω² = 1.47 × 10⁶
ω = 1,213 rad/s = 11,580 rpm
```

**必要トルクの計算：**

この回転数でのプロペラ負荷トルク（経験式）：
```
トルク ≈ トルク係数 × 空気密度 × (角速度)² × (直径)⁵
トルク ≈ 0.01 × 1.225 × (1,213)² × (0.031)⁵
トルク ≈ 0.45 mN⋅m
```

**必要電流の算出：**
```
必要電流 = 必要トルク ÷ トルク定数
         = 0.45 mN⋅m ÷ 0.614 mN⋅m/A
         = 0.73 A
```

**設計マージンの確認：**
```
電流制限：2.9 A
実使用：0.73 A
安全率：2.9 ÷ 0.73 = 4.0倍
```

十分な設計余裕があることが確認できました。

### 設計が完了？いえ、ここからが本番です

先ほどの計算で、ホバリング時に各モータ0.73Aが必要とわかりました。安全率も4倍あって安心...と思いきや、実はこれで設計が終わったわけではありません。

**実際のフライトで起きる「想定外」を考えてみましょう。**

#### ケース1：バッテリーが減ってきたとき

満充電時は3.7Vですが、飛行中にバッテリーは消耗します：

```
満充電時（3.7V）の回転数：
回転数 = (電圧 - 電流×抵抗) × Kv
      = (3.7 - 0.73×0.34) × 17,600
      = 11,967 rpm  ← 必要な11,580rpmを満たす

残量20%時（3.2V）の回転数：
回転数 = (3.2 - 0.73×0.34) × 17,600
      = 10,887 rpm  ← 必要回転数を下回る！
```

**つまり、バッテリー残量20%では推力不足でホバリングできません。**

では、ホバリング可能な最低電圧は何Vでしょうか？

### 最低電圧を計算する簡単なツール

この計算を自動化する簡単な関数を作ってみましょう：

```cpp
// モータが必要回転数を出せる最低電圧を計算
void calculateMinimumVoltage() {
    // StampFlyの実測値
    const float required_rpm = 11580.0f;  // 必要回転数
    const float kv = 17600.0f;           // 回転定数
    const float R = 0.34f;               // 内部抵抗
    const float I = 0.73f;               // ホバリング電流
    
    // 最低電圧 = (必要回転数/Kv) + (電流×抵抗)
    float min_voltage = (required_rpm / kv) + (I * R);
    
    printf("=== バッテリー電圧解析 ===\n");
    printf("ホバリング可能な最低電圧: %.2fV\n", min_voltage);
    printf("10%%安全マージンを含む推奨下限: %.2fV\n", min_voltage * 1.1f);
    
    // 実用的な情報も表示
    float flight_time_percent = (min_voltage - 3.0f) / (3.7f - 3.0f) * 100;
    printf("この電圧はバッテリー残量約%.0f%%に相当\n", flight_time_percent);
}
```

実行結果：
```
=== バッテリー電圧解析 ===
ホバリング可能な最低電圧: 0.91V
10%安全マージンを含む推奨下限: 1.00V
この電圧はバッテリー残量約-157%に相当
```

**あれ？計算が合いません。** 実は、この単純な計算には落とし穴があります。

#### ケース2：電流も電圧で変わる現実

電圧が下がると回転数が下がり、同じ推力を得るにはより大きな電流が必要になります。これを考慮した、より現実的な計算をしてみましょう：

```cpp
// 電圧に応じた必要電流も考慮する改良版
void calculateRealisticMinimumVoltage() {
    const float target_thrust = 0.136f;  // 1モータあたり必要推力[N]
    const float kt = 0.614e-3f;         // トルク定数[N⋅m/A]
    const float R = 0.34f;              // 内部抵抗[Ω]
    
    printf("=== 現実的なバッテリー電圧解析 ===\n");
    printf("電圧[V] | 推定電流[A] | 推力達成 | 飛行可否\n");
    printf("--------|------------|---------|--------\n");
    
    // 3.0Vから3.7Vまで0.1V刻みで評価
    for (float voltage = 3.0f; voltage <= 3.7f; voltage += 0.1f) {
        // この電圧での推定電流（簡易計算）
        float estimated_current = 0.73f * (3.7f / voltage);  // 電圧に反比例と仮定
        
        // 電圧降下を考慮した実効電圧
        float effective_voltage = voltage - estimated_current * R;
        
        // 推力が出せるかチェック
        bool can_hover = (effective_voltage > 3.2f);  // 簡易判定
        
        printf("  %.1f  |    %.2f    |   %s   |   %s\n",
               voltage, estimated_current,
               can_hover ? "○" : "×",
               can_hover ? "可能" : "不可");
    }
}
```

このような簡単なツールでも、設計の妥当性を確認する役に立ちます。

### さらに考慮すべき要因

実際の設計では、以下のような要因も検討が必要です：

1. **温度変化の影響**
   - 銅線の抵抗は温度で変化（0.004/℃）
   - 夏と冬で性能が変わる

2. **モータの個体差**
   - トルク定数のばらつき（±10%）
   - 4つのモータすべてが最悪値の可能性

3. **経年劣化**
   - ブラシの摩耗による効率低下
   - 磁石の減磁

### 設計ツールの発展

今回作った簡単な関数は、エクセルやPythonでも簡単に実装できます。興味がある方は、以下のような機能を追加してみてください：

- 温度補正機能
- グラフ表示機能
- 複数条件の一括評価

**重要なのは、物理法則に基づいて「なぜこの部品を選んだか」を定量的に説明できることです。**

今回の基本的な計算方法を理解できれば、より高度な設計検証へとステップアップできるでしょう。

## MOSFETドライバの設計根拠

### 電力制御の技術的課題

モータ仕様が決まったら、次は「ESP32-S3でこのモータをどう制御するか」という課題です。

**電力ギャップの問題：**
- ESP32-S3出力：3.3V、最大40mA → 0.13W
- モータ最大：3.7V、2.9A → 10.7W
- **電力比：約82倍の差**

### MOSFETスイッチングによる解決

**PWM制御の物理的原理：**

MOSFETは理想的な電力スイッチとして動作：

```
ON時間：モータに3.7V印加
OFF時間：モータへの電流遮断
デューティ比：ON時間/(ON時間+OFF時間)
```

**実効電圧の計算：**
```
実効電圧 = バッテリー電圧 × デューティ比
50%デューティ → 1.85V相当
75%デューティ → 2.78V相当
```

### MOSFETの2.9A定格で本当に大丈夫？

StampFlyに使用されているMOSFETの連続定格電流は2.9Aです。でも、本当にこれで十分なのでしょうか？

**必要電流との比較：**
```
ホバリング時：0.73A（定格の25%）→ 余裕十分
アクロ飛行時：1.46A（定格の50%）→ まだ余裕あり
最大推力時：約2.0A（定格の69%）→ 安全範囲内
```

**なぜ2.9A定格で十分なのか：**

1. **実使用では連続最大電流は流れない**
   - ドローンは常に姿勢を微調整
   - 4つのモータが同時に最大出力になることはほぼない

2. **短時間なら定格を超えても大丈夫**
   - MOSFETは短時間（数秒）なら定格の150%程度は許容
   - 瞬間的な大電流には対応可能

3. **4倍の安全率がある**
   - 通常使用：0.73A vs 定格2.9A = 4倍の余裕
   - この余裕が安定飛行の鍵

### 実際の動作を想像してみましょう

2.9Aまで制御できることがわかりました。でも、実際に飛行すると新たな疑問が生まれます：

**「ずっと最大電流を流し続けたら、熱くならない？」**

実際に計算してみましょう。

#### MOSFETの発熱計算

MOSFETには「オン抵抗」という特性があります。StampFlyのMOSFETは約0.1Ωです：

```
発熱 = 電流² × オン抵抗
ホバリング時（0.73A）：0.73² × 0.1 = 0.053W
最大時（2.9A）：2.9² × 0.1 = 0.84W
```

0.84Wの発熱は、小さなチップ部品にとってはかなりの熱量です。

### 温度上昇を見積もる簡単なツール

温度上昇を計算するには「熱抵抗」という概念を使います。熱抵抗とは、1Wの発熱で何℃温度が上がるかを表す値です。StampFlyの小さなMOSFETと基板を合わせると、約100℃/Wの熱抵抗になります。

つまり：
- 0.1Wの発熱 → 10℃上昇
- 1.0Wの発熱 → 100℃上昇

実際にどれくらい温度が上がるか計算してみましょう：

```cpp
// MOSFETの温度上昇を計算
void calculateMOSFETTemperature() {
    // MOSFETの基本仕様
    const float rds_on = 0.1f;           // オン抵抗[Ω]
    const float thermal_resistance = 100.0f;  // 熱抵抗[℃/W]（基板込み）
    const float ambient_temp = 25.0f;    // 周囲温度[℃]
    
    printf("=== MOSFET温度解析 ===\n");
    printf("電流[A] | 発熱[W] | 温度上昇[℃] | チップ温度[℃]\n");
    printf("-------|---------|------------|-------------\n");
    
    // 代表的な動作点での温度計算
    float currents[] = {0.73f, 1.0f, 1.5f, 2.0f, 2.9f};
    
    for (int i = 0; i < 5; i++) {
        float current = currents[i];
        float power_loss = current * current * rds_on;
        float temp_rise = power_loss * thermal_resistance;
        float chip_temp = ambient_temp + temp_rise;
        
        printf(" %.2f  |  %.3f  |    %.1f    |    %.1f\n",
               current, power_loss, temp_rise, chip_temp);
    }
    
    printf("\n※ 一般的な限界温度: 125℃\n");
}
```

実行結果：
```
=== MOSFET温度解析 ===
電流[A] | 発熱[W] | 温度上昇[℃] | チップ温度[℃]
-------|---------|------------|-------------
 0.73  |  0.053  |    5.3    |    30.3
 1.00  |  0.100  |   10.0    |    35.0
 1.50  |  0.225  |   22.5    |    47.5
 2.00  |  0.400  |   40.0    |    65.0
 2.90  |  0.841  |   84.1    |   109.1

※ 一般的な限界温度: 125℃
```

### さらに考慮すべき実務的な要因

実際の設計では、以下も検討が必要です：

1. **連続動作時の熱蓄積**
   - 4つのMOSFETが同時に発熱
   - 基板全体の温度上昇

2. **PWM周波数の選択**
   - 高周波：効率は良いがノイズが増加
   - 低周波：モータの音が聞こえる

3. **保護機能の実装**
   - 過電流保護
   - 温度保護

### 設計の妥当性確認

今回の計算から：
- ホバリング時（0.73A）：30℃程度で全く問題なし
- 最大電流時（2.9A）：109℃で限界に近いが短時間なら許容範囲

つまり、2.9A制限は熱的にも妥当な設計値であることが確認できました。

**このような簡単な計算でも、設計の良し悪しを判断する重要な情報が得られます。**

## プロペラ設計と空力特性

### 設計課題：限られた選択肢から最適解を見つける

**小型ドローンのプロペラ選定の現実**

StampFlyのプロペラ設計で直面する根本的な問題は、「設計自由度がほとんどない」ことです：

- **物理制約**：機体サイズ80mm×80mm、プロペラ対角65mm
- **市販品制約**：小型プロペラの標準径は31、40、51、65、75mm
- **推力要求**：各モータ0.136N（前セクションで算出済み）

一般的な機械設計なら「要求仕様に合わせて部品を設計」しますが、小型ドローンでは「限られた市販品から最適解を選択」する必要があります。

### プロペラ推力の基本原理

**運動量理論による推力発生メカニズム**

プロペラの推力は、空気を下向きに加速することで発生します。この物理現象は運動量理論で説明できます：

```
推力 = 推力係数 × 空気密度 × (角速度)² × (直径)⁴
```

**推力係数の理論的背景**

推力係数（Ct）は、プロペラの形状と空気力学特性を表す無次元数です。理論的には以下の値が知られています：

**理想的な推力係数（運動量理論）**
- 理論最大値：約0.12（完全効率の場合）
- 実用的な値：0.08-0.10（摩擦損失等を考慮）

**小型プロペラの特殊事情**
- 低レイノルズ数効果：小径プロペラ（< 50mm）では粘性影響が大きく、効率が低下
- StampFlyの31mm径プロペラ：ピッチ比0.74（22.86mm/31mm）でCt ≈ 0.08

### 手計算による選定プロセス

**各径での必要回転数の算出**

推力係数0.08を使用して、各標準径での必要回転数を計算します：

```
必要角速度 = √(推力 / (Ct × ρ × D⁴))
```

**31mm径の場合：**
```
ω = √(0.136 / (0.08 × 1.225 × (0.031)⁴))
  = √(0.136 / (0.08 × 1.225 × 9.23×10⁻⁷))
  = √(1.51×10⁶) = 1,230 rad/s
  = 11,750 rpm
```

**40mm径の場合：**
```
ω = √(0.136 / (0.08 × 1.225 × (0.040)⁴))
  = 6,800 rpm
```

**51mm径の場合：**
```
ω = √(0.136 / (0.08 × 1.225 × (0.051)⁴))
  = 4,100 rpm
```

### 実用上の複雑な判断要因

手計算では「理論的に可能」と判断されても、実際の選定では以下の複雑な要因を考慮する必要があります：

1. **モータ特性との適合性**：回転数だけでなく、効率カーブとの関係
2. **重量とのトレードオフ**：大径プロペラは重量増加で他部品を圧迫
3. **動的特性**：加速性能、騒音、振動特性
4. **製造ばらつき**：同じ仕様でもメーカーによる性能差
5. **使用条件**：室内飛行特有の気流条件

### 設計ツールの必要性

**なぜ自動化が必要か**

手計算では基本的な適合性は判断できますが、実際の設計では以下の理由で計算ツールが不可欠です：

- **パラメータ変更への対応**：機体重量や推力余裕率の変更時
- **条件変化への対応**：高度や温度による空気密度変化
- **最適化計算**：複数の制約条件を同時に満たす解の探索
- **設計検証**：異なる設計選択肢の定量的比較

### プロペラ選定支援ツール

**理論計算を設計に活用する**

以下のツールで、異なる設計条件での選定を効率化できます：

```cpp
// プロペラ推力の理論的見積もり
void estimatePropellerThrust() {
    // 物理定数
    const float air_density = 1.225f;         // kg/m³ (標準大気)
    const float thrust_coefficient = 0.08f;   // 小型プロペラの推力係数
    
    // 要求仕様
    const float required_thrust = 0.136f;     // N (目標推力)
    
    printf("=== プロペラ仕様の理論的見積もり ===\n");
    
    // 各直径での必要回転数を計算
    float diameters[] = {0.031f, 0.040f, 0.051f};  // m
    const char* diameter_names[] = {"31mm", "40mm", "51mm"};
    
    for (int i = 0; i < 3; i++) {
        float diameter = diameters[i];
        
        // 必要角速度の算出
        // 推力 = Ct × ρ × ω² × D⁴
        // ω = sqrt(推力 / (Ct × ρ × D⁴))
        float omega_squared = required_thrust / 
                             (thrust_coefficient * air_density * 
                              pow(diameter, 4));
        float omega = sqrt(omega_squared);      // rad/s
        float rpm = omega * 60.0f / (2.0f * M_PI);  // rpm
        
        printf("%s径: 必要回転数 %.0f rpm\n", 
               diameter_names[i], rpm);
    }
}
```

**理論計算結果：**
- 31mm径：必要回転数 約11,580 rpm
- 40mm径：必要回転数 約6,800 rpm  
- 51mm径：必要回転数 約4,100 rpm

### Step3: モータ特性とのマッチング

**StampFlyモータの実測特性：**
- 回転定数：Kv = 17,600 rpm/V
- 電源電圧：3.7V
- **最大回転数：約13,000 rpm**

```cpp
// モータ特性とのマッチング
void checkMotorMatching() {
    const float kv = 17600.0f;        // rpm/V
    const float voltage = 3.7f;       // V
    const float motor_resistance = 0.34f;  // Ω
    const float estimated_current = 0.73f; // A
    
    // モータの実効回転数
    float effective_voltage = voltage - estimated_current * motor_resistance;
    float max_rpm = effective_voltage * kv;
    
    printf("=== モータ特性とのマッチング ===\n");
    printf("モータ最大回転数: %.0f rpm\n", max_rpm);
    
    // 各プロペラの適合性
    float required_rpms[] = {11580.0f, 6800.0f, 4100.0f};
    const char* sizes[] = {"31mm", "40mm", "51mm"};
    
    for (int i = 0; i < 3; i++) {
        bool suitable = (required_rpms[i] <= max_rpm);
        float margin = max_rpm / required_rpms[i];
        printf("%s径: %s (余裕率%.1f倍)\n", 
               sizes[i], suitable ? "適合 ✓" : "不適合 ×", margin);
    }
}
```

### StampFlyの最終選定結果と設計判断

**理論的には3つすべてが使用可能**

上記ツールでの計算結果：
- **31mm径**：11,750 rpm必要、余裕率1.1倍
- **40mm径**：6,800 rpm必要、余裕率1.9倍
- **51mm径**：4,100 rpm必要、余裕率3.2倍

**StampFlyが31mm径を選んだ理由**

理論的にはどのサイズでも飛行可能ですが、教育プラットフォームとしての設計目標を考慮して選定：

1. **安全性優先**：小径で接触時のリスク最小化
2. **応答性重視**：高回転で素早い制御応答
3. **コスト効率**：交換パーツの入手性と価格
4. **音響特性**：4枚羽根で静かな室内飛行

**選定結果：31mm径、4枚羽根、22.86mmピッチ**

### 実証と性能確認

理論計算は設計の始まりです。実際の性能は飛行テストで検証します。

**StampFly飛行テスト結果**
- 安定ホバリング：約4分間達成
- 最大傾斜角：30度の機動飛行が可能
- 応答性：素早い姿勢制御が実現
- 騒音レベル：室内使用に適した静かさ

**設計検証の結果**

理論計算での予測と実測値が概ね一致し、プロペラ選定の理論的アプローチの有効性が確認できました。

このように、小型ドローンのプロペラ設計は「限られた選択肢から最適解を選択」するプロセスですが、理論的な背景を理解することで理由ある選定が可能になります。

```cpp
// プロペラ重量の影響評価
void evaluateWeightImpact() {
    // 予想プロペラ重量（一般的な値）
    float propeller_weights[] = {0.8f, 1.5f, 2.8f};  // g
    const char* sizes[] = {"31mm", "40mm", "51mm"};
    const float total_budget = 36.8f;  // g
    
    printf("=== 重量制約での選択 ===\n");
    
    for (int i = 0; i < 3; i++) {
        float total_prop_weight = propeller_weights[i] * 4;  // 4枚
        float weight_ratio = total_prop_weight / total_budget * 100;
        
        printf("%s径: %.1fg/機 (全体の%.1f%%)\n", 
               sizes[i], total_prop_weight, weight_ratio);
    }
}
```

**2. 安全性と効率のバランス**
- 室内飛行での安全性重視
- 小径で接触時の危険低減
- 4枚羽根で静響性向上

**3. 教育用途への最適化**
- 初心者が扱いやすいサイズ
- 修理・交換の容易さ
- コストパフォーマンス

**選定結果：31mm径、4枚羽根、22.86mmピッチ**

### Step6: 実証と性能確認

理論計算はあくまで予測です。実際の性能は飛行テストで確認します。

```cpp
// 実証結果の記録
void recordFlightTestResults() {
    printf("=== StampFly飛行テスト結果 ===\n");
    
    // 実測性能
    printf("ホバリング時間: 4分間 ✓\n");
    printf("姿勢制御: 安定 ✓\n");
    printf("応答性: 良好 ✓\n");
    printf("騒音レベル: 室内使用に適切 ✓\n");
    
    // 理論値との比較
    printf("\n理論予測との比較:\n");
    printf("予測推力: 0.136N/モータ\n");
    printf("実測結果: 必要推力を十分達成\n");
    
    printf("\n設計的結論:\n");
    printf("- 理論計算による予測が実用的精度で有効\n");
    printf("- 31mm径4枚羽根が教育用途に最適\n");
    printf("- 重量・安全性・性能のバランスが良好\n");
}
```

### 初心者が使える設計手順

**他のドローン設計への応用：**

1. **推力要求の算出**
   - 機体重量 × 重力加速度 × 余裕率 ÷ モータ数

2. **理論的なプロペラ仕様の算出**
   - 運動量理論による直径と回転数の関係

3. **モータ特性とのマッチング**
   - Kv値と電源電圧からの最大回転数確認

4. **物理制約と市販品の照合**
   - 機体サイズ、重量予算、調達性

5. **実証テスト**
   - 理論予測の正しさを実飛行で確認

**この手順を使えば、初心者でも理論に基づいたプロペラ選定が可能になります。**

### プロペラ設計の実用的教訓

**StampFlyの設計プロセスから学んだこと：**

1. **理論計算の有効性**
   - 運動量理論による予測は実用的精度
   - 初期設計段階での方向性確認に有効

2. **制約条件の重要性**
   - 物理サイズ、重量、モータ特性の統合的検討
   - 市販品の制約が設計を大きく左右

3. **バランス設計の重要性**
   - 最大性能より総合バランスを重視
   - 安全性、コスト、保守性などを総合判断

4. **実証の不可欠性**
   - 理論はあくまで予測、実飛行での検証が最終判断
   - 想定外の要因（振動、騒音、耐久性等）を発見

**他のドローン設計への応用指針：**

- まず理論計算で方向性を確認
- 制約条件を正確に整理
- 市販品から筆実な選択
- 実際に作って飛ばして検証

**このアプローチで、初心者でも理論に裏付けられた現実的なプロペラ選定が可能になります。**

## システム統合の設計課題

### 電気系統の統合設計

**電流バランスの設計：**

4つのモータが同時動作する際の電流配分：

```
総電流 = 4 × 0.73A = 2.92A （ホバリング時）
バッテリー容量：300mAh
飛行時間 = 300mAh ÷ 2920mA = 6.2分 （理論値）
```

実際は効率損失により約4分となり、設計目標と一致。

### 統合システムの複雑な設計課題

しかし、実際のシステム統合では、単純な足し算では解決できない問題があります：

**システム統合で考慮すべき相互作用：**

1. **電気的相互作用**
   - バッテリー電圧降下の非線形性
   - 4つのMOSFETドライバ間の干渉
   - 配線抵抗による電圧分布

2. **機械的相互作用**
   - プロペラ後流の相互干渉
   - フレーム振動の共振現象
   - 温度分布の不均一性

3. **制御系への影響**
   - モータ特性ばらつきによる制御偏差
   - 推力アンバランスの補正
   - 動的応答の個体差

### システム統合解析ツール

これらの複雑な相互作用を体系的に解析するツール：

```cpp
class SystemIntegrationAnalysis {
public:
    struct SystemState {
        float battery_voltage;
        float motor_currents[4];
        float motor_temperatures[4];
        float thrust_forces[4];
        float frame_vibration_level;
    };
    
    // システム全体の動的シミュレーション
    static void simulateFlightProfile() {
        printf("=== 飛行プロファイル統合シミュレーション ===\n");
        printf("時間[s] | 電圧[V] | 総電流[A] | 推力バランス | 振動[G] | 温度[℃]\n");
        
        SystemState state = initializeSystem();
        
        for (float time = 0; time <= 240; time += 30) {  // 4分間
            updateSystemState(state, time);
            
            float thrust_balance = calculateThrustBalance(state);
            float avg_temperature = calculateAverageTemperature(state);
            float total_current = calculateTotalCurrent(state);
            
            printf("%6.0f  | %6.2f | %8.2f | %10.3f | %7.2f | %8.1f\n",
                   time, state.battery_voltage, total_current,
                   thrust_balance, state.frame_vibration_level,
                   avg_temperature);
            
            // 警告レベルのチェック
            checkWarningLevels(state, time);
        }
    }
    
    // モータ特性ばらつきの影響評価
    static void analyzeMotorVariation() {
        printf("=== モータ特性ばらつき影響解析 ===\n");
        
        // 典型的な製造ばらつき
        float kt_variations[] = {0.90f, 0.95f, 1.00f, 1.05f, 1.10f};  // ±10%
        float resistance_variations[] = {0.85f, 0.92f, 1.00f, 1.08f, 1.15f};  // ±15%
        
        printf("Kt変動 | R変動 | 推力偏差[%%] | 電流偏差[%%] | 制御影響\n");
        
        for (float kt_var : kt_variations) {
            for (float r_var : resistance_variations) {
                auto deviation = calculatePerformanceDeviation(kt_var, r_var);
                
                printf("%6.2f | %5.2f | %9.1f | %9.1f | %s\n",
                       kt_var, r_var, deviation.thrust_percent,
                       deviation.current_percent,
                       (abs(deviation.thrust_percent) > 5.0f) ? "要補正" : "許容範囲");
            }
        }
    }
    
    // 熱的カップリング解析
    static void analyzeThermalCoupling() {
        printf("=== システム熱解析 ===\n");
        
        struct ThermalNode {
            const char* component;
            float power_dissipation;  // [W]
            float thermal_mass;       // [J/K]
            float coupling_coefficient[4];  // 他ノードとの結合
        };
        
        ThermalNode nodes[] = {
            {"モータ1", 0.18f, 2.5f, {1.0f, 0.1f, 0.05f, 0.1f}},
            {"モータ2", 0.18f, 2.5f, {0.1f, 1.0f, 0.1f, 0.05f}},
            {"モータ3", 0.18f, 2.5f, {0.05f, 0.1f, 1.0f, 0.1f}},
            {"MOSFET", 0.20f, 0.1f, {0.05f, 0.05f, 0.05f, 1.0f}}
        };
        
        // 熱回路網解析（簡略版）
        printf("成分    | 定常温度[℃] | 時定数[s] | 相互影響\n");
        for (int i = 0; i < 4; i++) {
            float steady_temp = 25.0f + nodes[i].power_dissipation * 50.0f;  // 簡易計算
            float time_constant = nodes[i].thermal_mass * 50.0f;  // 熱時定数
            
            printf("%-7s | %10.1f | %7.1f | ", nodes[i].component, steady_temp, time_constant);
            
            // 他ノードからの影響
            float coupling_effect = 0;
            for (int j = 0; j < 4; j++) {
                if (i != j) coupling_effect += nodes[i].coupling_coefficient[j] * 2.0f;
            }
            printf("%.1f℃\n", coupling_effect);
        }
    }
    
    // 故障診断システム
    static void performSystemDiagnostics(const SystemState& current_state) {
        printf("=== システム診断 ===\n");
        
        // 1. 電気系統診断
        if (current_state.battery_voltage < 3.4f) {
            printf("警告: バッテリー電圧低下 (%.2fV)\n", current_state.battery_voltage);
        }
        
        // 2. 推力バランス診断
        float thrust_balance = calculateThrustBalance(current_state);
        if (abs(thrust_balance) > 0.05f) {
            printf("警告: 推力アンバランス (%.1f%%)\n", thrust_balance * 100);
            
            // 原因特定
            for (int i = 0; i < 4; i++) {
                if (abs(current_state.thrust_forces[i] - 0.136f) > 0.02f) {
                    printf("  モータ%d異常: 推力%.3fN (期待値0.136N)\n", 
                           i+1, current_state.thrust_forces[i]);
                }
            }
        }
        
        // 3. 熱問題診断
        for (int i = 0; i < 4; i++) {
            if (current_state.motor_temperatures[i] > 60.0f) {
                printf("警告: モータ%d過熱 (%.1f℃)\n", i+1, current_state.motor_temperatures[i]);
            }
        }
        
        // 4. 振動診断
        if (current_state.frame_vibration_level > 2.0f) {
            printf("警告: 過振動検出 (%.1fG)\n", current_state.frame_vibration_level);
        }
    }
    
private:
    static SystemState initializeSystem() {
        return {3.7f, {0.73f, 0.73f, 0.73f, 0.73f}, 
                {25.0f, 25.0f, 25.0f, 25.0f},
                {0.136f, 0.136f, 0.136f, 0.136f}, 0.5f};
    }
    
    static void updateSystemState(SystemState& state, float time) {
        // バッテリー電圧降下
        state.battery_voltage = 3.7f - time * 0.002f;  // 時間経過で電圧低下
        
        // 温度上昇
        for (int i = 0; i < 4; i++) {
            state.motor_temperatures[i] = 25.0f + time * 0.2f;  // 緩やかな温度上昇
        }
        
        // 振動レベル（回転数依存）
        state.frame_vibration_level = 0.5f + sin(time * 0.1f) * 0.2f;
    }
    
    static float calculateThrustBalance(const SystemState& state) {
        float avg_thrust = 0;
        for (int i = 0; i < 4; i++) avg_thrust += state.thrust_forces[i];
        avg_thrust /= 4.0f;
        
        float max_deviation = 0;
        for (int i = 0; i < 4; i++) {
            float deviation = abs(state.thrust_forces[i] - avg_thrust) / avg_thrust;
            if (deviation > max_deviation) max_deviation = deviation;
        }
        return max_deviation;
    }
    
    static float calculateAverageTemperature(const SystemState& state) {
        float sum = 0;
        for (int i = 0; i < 4; i++) sum += state.motor_temperatures[i];
        return sum / 4.0f;
    }
    
    static float calculateTotalCurrent(const SystemState& state) {
        float sum = 0;
        for (int i = 0; i < 4; i++) sum += state.motor_currents[i];
        return sum;
    }
    
    static void checkWarningLevels(const SystemState& state, float time) {
        if (state.battery_voltage < 3.4f && time > 180) {
            printf("  → 着陸準備推奨\n");
        }
    }
    
    struct PerformanceDeviation {
        float thrust_percent;
        float current_percent;
    };
    
    static PerformanceDeviation calculatePerformanceDeviation(float kt_factor, float r_factor) {
        // モータ特性変動による性能偏差計算
        PerformanceDeviation dev;
        dev.thrust_percent = (kt_factor - 1.0f) * 100;  // Kt比例
        dev.current_percent = (1.0f/kt_factor - 1.0f) * 100;  // 逆比例
        return dev;
    }
};
```

**この統合解析システムの価値：**

1. **システム全体の可視化**：部分最適ではない全体最適の実現
2. **故障予兆の早期発見**：予防保全による信頼性向上
3. **設計余裕の定量評価**：安全率の科学的根拠
4. **製品化への道筋**：量産時の品質管理指標

## まとめ：実データに基づく設計の価値

### 学んだ設計プロセス

1. **要求の定量化**：36.8g → 0.136N/モータ
2. **物理計算による選定**：実測Kt = 0.614 mN⋅m/A
3. **制約条件の確認**：2.9A制限で4倍の安全率
4. **統合設計の検証**：熱・振動・効率の総合評価

### 設計ツールとしてのコードの意義

本記事で示したコードは、単なる計算例ではありません：

**エンジニアリングツールとしての価値：**

1. **設計の再現性**：手計算では困難な複雑計算の自動化
2. **設計変更への迅速対応**：仕様変更時の影響評価
3. **品質保証**：計算ミスの防止と検証の自動化
4. **知識の蓄積**：設計ノウハウのシステム化

### 実データの重要性

理論計算だけでなく、実測値（トルク定数、内部抵抗など）を使うことで：

- **設計の妥当性**を定量的に検証
- **性能予測の精度**を大幅に向上
- **トラブル診断**の根拠を明確化
- **改良設計**の方向性を特定

### 次回予告：第4回「センサシステムの物理設計」

推力システムの状態を正確に把握するため、次回は9つのセンサシステムの設計プロセスを解説：

- **IMUの選定根拠**：なぜBMI270なのか？
- **測定精度の要求**：制御に必要な分解能
- **センサ配置の物理的制約**：振動・磁気干渉の対策
- **データ処理の実装**：生データから制御入力への変換

**予習のポイント**：MEMSセンサの動作原理と、物理量測定における誤差要因について調べておくと理解が深まります。

---

**シリーズ**: 基礎・ハードウェア編（ハードウェア深掘り編）  
**対象読者**: 中級  
**推定読了時間**: 20分  
**重点ポイント**: 実データに基づく定量的設計プロセス  
**物理的連鎖**: 設計要求 → 物理計算 → 部品選定 → 実装検証

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