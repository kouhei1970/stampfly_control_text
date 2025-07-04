#\!/bin/bash

# V2.0記事リスト（モータダイナミクス重視）
declare -a v2_articles=(
    "01:01_drone_introduction:ドローンの世界への扉:基礎・ハードウェア編:全レベル:なし（概念解説）"
    "02:02_stampfly_hardware:StampFlyハードウェア完全解説:基礎・ハードウェア編:初級〜中級:M5StampFlyHardware"
    "03:03_motor_esc_propeller:モータ・ESC・プロペラシステム:基礎・ハードウェア編:中級:MotorESCSystem, PropellerCharacteristics"
    "04:04_sensor_configuration:センサ設定と状態監視:基礎・ハードウェア編:中級:BMI270Config, SensorFusion"
    "05:05_powertrain_integration:パワートレイン統合システム:基礎・ハードウェア編:中級〜上級:PowertrainSystem, BatteryManagement"
    "06:06_realtime_foundation:リアルタイムシステム基盤:基礎・ハードウェア編:上級:FreeRTOSTaskManager"
    "07:07_thrust_torque_science:推力とトルクの科学:物理モデリング編:中級:ThrustCalculator, TorqueModel"
    "08:08_motor_dynamics_analysis:モータダイナミクスの詳細解析:物理モデリング編:上級:MotorDynamics, ESCController"
    "09:09_forces_and_moments:機体に働く力とモーメント:物理モデリング編:中級〜上級:MulticopterForces, AerodynamicEffects"
    "10:10_rigid_body_mathematics:剛体運動の数学:物理モデリング編:上級:RigidBodyDynamics, Quaternion"
    "11:11_coordinate_systems:座標系と状態表現:物理モデリング編:中級:CoordinateTransform, MulticopterState"
    "12:12_environmental_disturbances:環境外乱とモデル化:物理モデリング編:中級〜上級:EnvironmentalFactors, DisturbanceModel"
    "13:13_control_fundamentals:制御系設計の基礎:制御システム編:初級〜中級:ControlSystemFoundation"
    "14:14_pid_theory_implementation:PID制御の理論と実装:制御システム編:中級:AdvancedPIDController"
    "15:15_motor_response_control:モータ応答を考慮した制御設計:制御システム編:上級:MotorCompensatedController"
    "16:16_cascade_control:カスケード制御システム:制御システム編:上級:CascadeController"
    "17:17_sensor_fusion:センサフュージョンと状態推定:制御システム編:中級〜上級:ExtendedKalmanFilter, MadgwickFilter"
    "18:18_digital_control:ディジタル制御実装:制御システム編:上級:DiscreteController"
    "19:19_hierarchical_integration:階層制御の統合:統合制御実装編:上級:FlightController"
    "20:20_motor_mixing_allocation:モータミキシングと配分:統合制御実装編:上級:AdvancedMotorMixer"
    "21:21_safety_failsafe:安全システムとフェイルセーフ:統合制御実装編:中級〜上級:SafetyMonitor"
    "22:22_realtime_control:リアルタイム制御実装:統合制御実装編:上級:RealtimeController"
    "23:23_communication_remote:通信と遠隔制御:統合制御実装編:中級〜上級:ESP_NOW_Controller"
    "24:24_simulation_validation:シミュレーションとモデル検証:テスト・評価編:上級:MulticopterSimulator"
    "25:25_system_identification:システム同定とパラメータ推定:テスト・評価編:上級:SystemID"
    "26:26_performance_optimization:性能評価と最適化:テスト・評価編:上級:PerformanceAnalyzer"
    "27:27_fault_diagnosis:故障診断と予測保守:テスト・評価編:上級:FaultDiagnostics"
    "28:28_ai_machine_learning:AI・機械学習制御:先端技術編:上級:AIController"
    "29:29_adaptive_robust:適応制御とロバスト制御:先端技術編:上級:AdaptiveController"
    "30:30_swarm_cooperative:群制御と協調飛行:先端技術編:上級:SwarmController"
)

for article in "${v2_articles[@]}"; do
    IFS=':' read -r num filename title series level classes <<< "$article"
    
    # 読了時間を計算
    case $level in
        "全レベル"|"初級〜中級") time="8分" ;;
        "中級") time="10分" ;;
        "中級〜上級") time="12分" ;;
        "上級") time="15分" ;;
        *) time="10分" ;;
    esac
    
    # 重点ポイントを設定
    key_point=""
    case $num in
        "03") key_point="電気→機械→空力エネルギー変換の詳細" ;;
        "08") key_point="PWM指令から実際の推力発生までの動的応答" ;;
        "15") key_point="制御帯域とモータ応答の関係" ;;
        "20") key_point="物理制約下での制御力配分" ;;
        *) key_point="前後記事との物理的連鎖" ;;
    esac
    
    cat > "blog_articles_v2/${filename}.md" << ARTICLE_EOF
# ${title} - StampFly制御システム第${num}回

## この記事で学ぶこと（2分で読める概要）

## 基礎知識の整理（初中級者向け詳細解説）

## ${classes}の設計思想

## 実装詳解（コード中心）

## よくある問題と対処法

## 発展的内容（上級者向け）

## まとめと次回予告

---

**シリーズ**: ${series}  
**対象読者**: ${level}  
**推定読了時間**: ${time}  
**メインクラス**: \`${classes}\`  
**重点ポイント**: ${key_point}  
**物理的連鎖**: PWM → ESC → モータ → プロペラ → 推力 → 機体運動
ARTICLE_EOF

    echo "Generated V2: ${filename}.md"
done

echo ""
echo "=== V2.0記事生成完了 ==="
echo "特徴："
echo "- モータダイナミクス重視（8記事）"
echo "- 物理的連鎖を明確化"
echo "- PWM→推力→運動の完全理解"
echo "- 制御系設計への影響考慮"
