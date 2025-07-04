# StampFly 仕様・ピンアサインまとめ

## 🛠 概要

- **製品名**：StampFly（M5Stamp Fly / SKU: K138）
- **タイプ**：ESP32-S3搭載 超小型クアッドコプター
- **サイズ**：81.5 × 81.5 × 31 mm
- **重量**：36.8g（機体のみ）
- **コントローラ**：M5StampS3（ESP32-S3、Wi-Fi、USB-OTG）
- **センサー構成**：
  - IMU：BMI270（6軸）
  - 磁気センサ：BMM150
  - 気圧センサ：BMP280
  - 距離センサ：VL53L3CX ×2（前方・下方）
  - 光学フローセンサ：PMW3901MB-TXQT
  - 電圧・電流モニタ：INA3221AIRGVR（@0x40）
- **出力・通知**：
  - パッシブブザー
  - WS2812 RGB LED（アドレス指定可能）
- **電源**：
  - バッテリー：300mAh HVリチウム（4.35V）
  - 飛行時間：約4分
- **インターフェース**：
  - Grove端子 ×2（I2C/UART）
  - SPI, I2C, UART, PWM出力
- **開発環境**：
  - Arduino / ESP-IDF 対応
  - Atom JoystickとESP-NOW連携による手動/自動制御

---

## 📌 ピンアサイン一覧

### I2C（共通バス）

| デバイス       | SDA | SCL |
|----------------|-----|-----|
| BMP280         | G3  | G4  |
| BMM150         | G3  | G4  |
| VL53L3CX ×2    | G3  | G4  |
| INA3221        | G3  | G4  |

---

### SPI

| デバイス       | MOSI | MISO | SCK | CS  |
|----------------|------|------|-----|-----|
| BMI270         | G14  | G43  | G44 | G46 |
| PMW3901MB-TXQT | G14  | G43  | G44 | G12 |

---

### Grove端子

| コネクタ | 機能 | SDA / TX | SCL / RX |
|----------|------|----------|----------|
| 赤（A） | I2C  | G13      | G15      |
| 黒（B） | UART | G1       | G2       |

**注記**: 黒Grove（UART）はS.BUS、DSM2、PPM等の受信機接続にも使用可能

---

### ブザー・LED

| デバイス | ピン |
|----------|------|
| ブザー（BEEP） | G12 |
| WS2812 RGB LED（外部） | G39 |
| RGB LED（M5StampS3内蔵） | G21 |

---

## ⚙ モーター接続・回転方向

### PWMピンアサイン

| モーター位置 | ピン番号 | 回転方向（正しい） |
|--------------|----------|---------------------|
| M1（右前）   | G6       | 反時計回り（CCW）   |
| M2（左前）   | G5       | 時計回り（CW）       |
| M3（左後）   | G8       | 反時計回り（CCW）   |
| M4（右後）   | G7       | 時計回り（CW）       |

### モーター回転方向図（上から見た図）

     前方
     ↑
 [M2]    [M1]
 CW       CCW
   \      /
    \____/
    /    \
 [M3]    [M4]
 CCW       CW
     ↓
    後方


---

## 📝 注意事項

- **磁気干渉に注意**：BMM150 は磁場に敏感なため、磁石や鉄部品の近くを避けて設置すること。
- **プロペラのCW/CCW配置**：プロペラの方向も正しく取り付ける必要があります。誤って逆向きに取り付けると、離陸できず転倒します。
- **初期ファームウェア**：出荷時にはデバッグ用ファームが書き込まれており、Atom Joystickとの連携や各種センサの確認が可能です。

---

## 📚 参考リンク

- [StampFly 公式ドキュメント](https://docs.m5stack.com/en/app/Stamp%20Fly)
- [M5Stamp S3 製品ページ](https://docs.m5stack.com/en/core/stampS3)
- [M5Stack EasyLoader](https://docs.m5stack.com/en/quick_start/stampfly/arduino)
- [StampFly Farmware](https://github.com/kouhei1970/M5StampFly)

