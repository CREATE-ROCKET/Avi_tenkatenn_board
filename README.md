# 99L Ground Station

ESP32とE220を使用する99L地上局受信機です。対象branchは`vault`です。

## Architecture

- `src/protocol.*`: Vault 04/04aのLSB-first compact packet、XOR、semantic error、11 byte uplink、pending transaction IDを扱うpure codecです。
- `src/decode.*`: E220 UARTの所有taskです。header別の可変長を判定し、XOR検証済みpacketを16件のbounded queueへ渡します。UART末尾のRSSIはapplication packetの外側として扱います。
- `src/main.cpp`: packet表示task、uplink送信task、console入力を担当します。送信taskだけがtransaction ID stateとUART TXを所有します。
- E220 PHY設定、pin、LED、設定modeは従来構成を維持しています。

受信packetはA0、A1〜A3、A4、A5、A6、B0、B1です。E220固定送信prefix `00 00 04`はGround側UARTへ届かない前提で、XOR対象に含めません。bit packingはLSB-firstです。

## Build / flash / run

```sh
/home/hotaru/.platformio/penv/bin/pio run
/home/hotaru/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB<N>
/home/hotaru/.platformio/penv/bin/pio device monitor -p /dev/ttyUSB<N> -b 115200
```

`<N>`を推測しないでください。`udevadm info --name=/dev/ttyUSB<N>`と既存boot logでGround Stationを識別してからflashします。

E220設定を書き込む場合だけ`src/config.h`の`BOOT_MODE`を`LoRaSetup`へ変更し、書込み後は必ず`Communication`へ戻して再flashします。設定値、pin、LoRa channelは既存値を維持しています。

## Command console

115200 bpsのUSB consoleで以下を使用します。数値はdecimalまたは`0x`付きhexです。

```text
g <command> [arg0 ... arg5]
ae
le
local <command> [arg0 ... arg5]
time <request_id> <unix_seconds> <milliseconds>
```

- `g`: Mission generic command。unused argは省略すると0です。
- `ae`: ActuatorEmergencyStop専用frame。
- `le`: LiftoffDetectionEmergencyStop専用frame。
- `local`: ComBoard local command。既存logging/GNSS codeは`0x6c/0x6d/0x67/0x68`です。
- `time`: B1に表示されたrequest IDへGround sourceの時刻を応答します。

transaction ID 0は使用せず、同時pendingは16件までです。B0のAcceptedでは保持し、Completed/Rejected/Failedで解放します。送信失敗時もIDを解放します。

## Test

```sh
sh test/run_host_tests.sh
/home/hotaru/.platformio/penv/bin/pio run
```

host testはMission/ComBoardとbyte-identicalな`testdata/99l_protocol_golden_vectors.txt`を読み、packet、bit境界、signed値、reserved/error、padding、checksum、uplink、transaction lifecycleを検証します。

## Hardware assumptions / known limitations

- E220はUART 115200 bps、SF8/BW125、CH4、append-RSSIの既存設定を前提とします。
- A0 status bitとA6/B0/B1 layout、requested torque scaleはVaultの実装仮定台帳に記録した暫定値です。
- packet queue overflow時はdrop countをconsoleへ表示します。永続logはGround Stationの責務外です。
- RSSI欠落時もXOR検証済みapplication packetを破棄せず、RSSI unavailableとして表示します。
- LEDは受信したMission packet headerに基づく表示です。A0 statusの任意bitをactuator commandとして使用しません。
