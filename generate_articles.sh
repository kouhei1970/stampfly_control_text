#\!/bin/bash

# 記事情報の配列（記事番号:ファイル名:タイトル:シリーズ:対象読者:メインクラス）
declare -a articles=(
    "04:04_thrust_calculation:推力計算の科学:航空工学編:初級〜中級:ThrustCalculator"
    "05:05_forces_and_moments:機体に働く力とモーメント:航空工学編:中級:MulticopterForces, MulticopterMoments"
    "06:06_motor_control_mixing:モータ制御とミキシング:航空工学編:中級〜上級:MotorController, AdvancedMotorMixer"
    "07:07_propeller_analysis:プロペラ性能解析:航空工学編:上級:PropellerPerformanceAnalyzer"
    "08:08_environmental_factors:環境要因と外乱対応:航空工学編:中級〜上級:EnvironmentalFactors"
    "09:09_state_coordinates:状態表現と座標系:剛体力学編:初級〜中級:MulticopterState, CoordinateTransform"
    "10:10_quaternion_mastery:クォータニオンマスター:剛体力学編:中級〜上級:Quaternion, QuaternionIntegrator"
    "11:11_euler_angles:オイラー角とその落とし穴:剛体力学編:中級:EulerAngles, EulerAngleIntegrator"
    "12:12_rigid_body_dynamics:剛体運動の力学:剛体力学編:中級〜上級:TranslationalDynamics, RotationalDynamics"
    "13:13_numerical_integration:数値積分と安定性:剛体力学編:上級:RungeKutta4Integrator"
    "14:14_complete_simulation:完全シミュレーション構築:剛体力学編:上級:MulticopterSimulation"
    "15:15_pid_deep_dive:PID制御の深層理解:制御理論編:初級〜上級:AdvancedPIDController, PIDComponentAnalyzer"
    "16:16_cascade_control:カスケード制御システム:制御理論編:中級〜上級:MultilayerCascadeController, CascadeDesignHelper"
    "17:17_state_space_control:状態空間と現代制御:制御理論編:上級:AltitudeStateSpace, LQRController"
    "18:18_digital_control:ディジタル制御実装:制御理論編:上級:DiscreteController, AntiAliasingFilter"
    "19:19_sensor_fusion:センサフュージョンの実践:実装統合編:中級〜上級:ComplementaryFilter, MadgwickFilter, ExtendedKalmanFilter"
    "20:20_hierarchical_control:階層制御の統合実装:実装統合編:上級:RateController, AttitudeController, VelocityController, PositionController"
    "21:21_freertos_tasks:FreeRTOSタスク設計:実装統合編:上級:AdvancedMultiRateController"
    "22:22_safety_system:安全システムの実装:実装統合編:中級〜上級:SafetyMonitor, EmergencyProcedures"
    "23:23_espnow_communication:ESP-NOW通信実装:実装統合編:中級〜上級:ESP-NOW実装群"
    "24:24_simulation_hil:シミュレーションとHIL:テスト・評価編:上級:DroneSimulator, HILSimulation, AutomatedTestFramework"
    "25:25_system_identification:システム同定と調整:テスト・評価編:上級:SystemIdentification, PIDTuner"
    "26:26_performance_optimization:性能評価と最適化:テスト・評価編:上級:PerformanceAnalyzer, RealTimeMonitor"
    "27:27_ai_machine_learning:AI・機械学習制御:先端技術編:上級:AdvancedAIFramework, NextGenerationControlSystem"
    "28:28_energy_optimization:エネルギー最適化技術:先端技術編:上級:AdvancedEnergyOptimizer"
    "29:29_swarm_intelligence:群制御と協調飛行:先端技術編:上級:AdvancedSwarmIntelligence"
    "30:30_robust_fault_diagnosis:ロバスト制御と故障診断:先端技術編:上級:AdvancedRobustController, AdvancedFaultDiagnosisSystem"
)

# 各記事ファイルを生成
for article in "${articles[@]}"; do
    IFS=':' read -r num filename title series level classes <<< "$article"
    
    # 読了時間を計算（レベルによって調整）
    case $level in
        "初級〜中級"|"初級") time="8分" ;;
        "中級") time="10分" ;;
        "中級〜上級") time="12分" ;;
        "上級") time="15分" ;;
        *) time="10分" ;;
    esac
    
    # 前後の記事リンクを生成
    prev_num=$((num - 1))
    next_num=$((num + 1))
    
    links=""
    if [ $prev_num -ge 1 ]; then
        links="[第${prev_num}回]($(printf "%02d" $prev_num)_*.md)"
    fi
    if [ $next_num -le 30 ]; then
        if [ -n "$links" ]; then
            links="$links | "
        fi
        links="${links}[第${next_num}回]($(printf "%02d" $next_num)_*.md)"
    fi
    
    # ファイル生成
    cat > "blog_articles/${filename}.md" << ARTICLE_EOF
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
**関連記事**: ${links}
ARTICLE_EOF

    echo "Generated: ${filename}.md"
done

echo "All article templates generated successfully\!"
