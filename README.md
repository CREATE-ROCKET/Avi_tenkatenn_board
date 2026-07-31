# receicer_tennkaten

ESP32とE220 LoRaモジュールを使用し、テレメトリを受信・表示するPlatformIOプロジェクトです。
通常通信モードに加えて、E220へ設定を書き込むための設定モードを備えています。

## ファイル構成

- `src/main.cpp`: Arduinoの`setup()` / `loop()`、起動モードの分岐、コマンド送信、LED表示、テレメトリ表示を担当します。
- `src/config.h`: 起動モード、ピン番号、E220設定値、フレーム定数、`TelemetryData`を定義します。
- `src/decode.h`: デコード処理を開始し、最新テレメトリを取得するための公開APIです。
- `src/decode.cpp`: LoRa受信、フレーム検出、チェックサム検証、テレメトリ変換と共有を担当します。
- `platformio.ini`: 対象ボード、シリアル速度などのPlatformIO設定です。

## 通常通信モード

`src/config.h`を次の設定にします。

```cpp
constexpr BootMode BOOT_MODE = BootMode::Communication;
```

起動後はLoRaから受信したフレームを検証・デコードし、GNSS情報とRSSIをシリアルモニタへ表示します。
シリアルモニタから入力した1文字は、E220の固定送信形式で送信されます。

## LoRa設定モード

E220の設定を書き込む場合は、`src/config.h`を次のように変更します。

```cpp
constexpr BootMode BOOT_MODE = BootMode::LoRaSetup;
```

設定手順:

1. `BOOT_MODE`を`BootMode::LoRaSetup`へ変更します。
2. 使用するボードへファームウェアを書き込みます。
3. 115200 bpsのシリアルモニタで設定結果を確認します。
4. `BOOT_MODE`を`BootMode::Communication`へ戻します。
5. 通常運用用ファームウェアを再度書き込みます。

設定モードではM0/M1をHIGHにして、`settingCmd`をE220へ送信します。通常の受信タスクとコマンド送信は動作しません。
設定値、LoRaアドレス、チャンネル、ピン番号を変更する場合は`src/config.h`を編集します。

## LED

- `top_led`: テレメトリの頂点検知状態で点灯します。
- `liftoff_led`: リフトオフ状態で点灯します。
- `control_led`: 制御状態で点灯します。
- `update_led`: 正常なテレメトリ受信で点灯し、5秒間受信がない場合に消灯します。

## PlatformIO環境

- `esp32doit-devkit-v1`: DOIT ESP32 DEVKIT V1
