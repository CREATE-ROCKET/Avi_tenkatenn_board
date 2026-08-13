# 99L Ground Station

ESP32とE220を使用する99L地上局受信機です。対象branchは`vault`です。

## Architecture

- `src/protocol.*`: Vault 04/04aのLSB-first compact packet、XOR、semantic error、11 byte uplink、pending transaction IDを扱うpure codecです。
- `src/decode.*`: E220 UARTの所有taskです。header別の可変長を判定し、XOR検証済みpacketを16件のbounded queueへ渡します。UART末尾のRSSIはapplication packetの外側として扱います。
- `src/main.cpp`: packet表示task、uplink送信task、console入力を担当します。UART TXは送信taskが所有し、transaction stateは送信taskの確保/解放と受信taskの終端result反映をmutexで直列化します。
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

設定を変更せず読み戻す場合は`BOOT_MODE`=`LoRaReadback`を使用します。このmodeは`C1 00 08`だけを送信し、raw応答を表示してcommunication modeへ戻します。診断後は`BOOT_MODE`=`Communication`でproduction firmwareを再flashしてください。

## Command console

115200 bpsのUSB consoleで以下を使用します。数値はdecimalまたは`0x`付きhexです。

```text
g <command> [arg0 ... arg5]
ae
le
local <command> [arg0 ... arg5]
time <request_id> <unix_seconds> <milliseconds>
release <transaction_id>
```

- `g`: Mission generic command。unused argは省略すると0です。
- `ae`: ActuatorEmergencyStop専用frame。
- `le`: LiftoffDetectionEmergencyStop専用frame。
- `local`: ComBoard local command。既存logging/GNSS codeは`0x6c/0x6d/0x67/0x68`です。
- `time`: B1に表示されたrequest IDへGround sourceの時刻を応答します。
- `release`: B0終端結果を受信できなかったpending IDを、operator確認後に解放します。自動再送はせず、再入力したcommandには新しいIDを割り当てます。

transaction ID 0は使用せず、同時pendingは16件までです。通常commandは14件までとし、Actuator/Liftoff Emergency用に2枠を予約します。B0のAcceptedでは保持し、Completed/Rejected/Failedで解放します。送信失敗時もIDを解放します。B0を喪失したpendingには自動timeoutを設けていないため、Mission/ComBoardの状態を確認してから`release`を実行してください。

通常uplinkは、直前のdownlinkをdecodeした通知を待ってから送信します。ComBoardのpost-TX RX windowと送信時刻を合わせるためで、2秒以内にdownlinkがなければ送信せず、確保したtransaction IDを解放します。Actuator/Liftoff Emergencyはtask notificationで通常queueを迂回し、freshなRX windowを最大2秒待ってから送信します。downlinkが来ない場合もtimeout後に送信を試みます。ただし、進行中の1回のUART送信自体は中断しません。

## Test

```sh
sh test/run_host_tests.sh
/home/hotaru/.platformio/penv/bin/pio run
```

host testはMission/ComBoardとbyte-identicalな`testdata/99l_protocol_golden_vectors.txt`を読み、packet、bit境界、signed値、reserved/error、padding、checksum、uplink、transaction lifecycleを検証します。

## Hardware validation (2026-08-14)

Mission Board、Communication Board、Ground Boardの3基板を接続し、Groundは`/dev/ttyUSB0`の`Communication`モードで検証しました。

- **PASS**: production boot、decode task、command taskはpanic/resetなしで継続しました。
- **PASS**: read-only診断でE220に`C1 00 08`だけを送り、`C1 00 08 00 00 EC 81 04 C3 00 00`を読み戻しました。正式基準byteとは一致しますが、module型番は未確定のためfieldの意味はraw一致までの評価です。設定writeは行っていません。
- **PASS**: 130秒captureで203 packetを受信しました。内訳はA0=116、B0=3、B1=84、受信間隔は最小0.598 s/平均0.642 s/最大1.320 s、1秒超gap=1、5秒超gap=0です。RSSIは203/203 packetにあり、このcaptureでは最小/平均/最大すべて-107 dBmでした。decode errorとGround packet queue overflowはありませんでした。
- **PASS**: 最終productionの65秒同時captureで103 packet（A0=38、B1=63、B0=2）、1秒超gap 1、5秒超gap 0、RSSI missing 0、panic/reset loopなしを確認しました。`g 0x7F`と`le`はどちらもuplink送信後0.360秒で期待の終端B0を返し、pending/unmatched warningは0です。rawは`/tmp/99l_hwtest_20260814_G8Eh5f/final_production_65s_v5/`です。
- **PASS**: `g 0x7F` → Rejected/NotSupported、`g 0x02` → Rejected/InvalidState、`le` → Rejected/InvalidState、`ae` → Completed/None、GNSS/logging/recoveryの安全側local commandでGround→LoRa→ComBoard→CAN→Mission→CAN→ComBoard→LoRa→Groundを確認しました。未知generic commandの入力から終端B0までの最短実測latencyは0.515 sでした。
- **PASS**: 実機で使用したtransaction ID 4〜14はすべて0以外で、Acceptedと終端B0は同じIDで受信しました。終端B0でのID解放はhost testで検証済みです。
- **NOT_EVALUATED**: 実機でID wrap/exhaustionまで送るpending leak試験と、unknown/malformed B0で別IDを解放しないことは実施していません。`release`は必要なpending喪失がなかったため実行していません。
- **NOT_IMPLEMENTED / SPEC_GAP**: 最新VaultのUSB line protocol v1が定義する`@RX` / `@TX` / `@FRAG` / `@SYS`と`#`行は未実装です。今回は既存pretty-printをhost側でtimestamp付きcaptureして検証しました。

## Hardware assumptions / known limitations

- E220はUART 115200 bps、SF8/BW125、CH4、append-RSSIの既存設定を前提とします。
- A0 status bitとA6/B0/B1 layout、requested torque scaleはVaultの実装仮定台帳に記録した暫定値です。
- A0のfin/para mode 6〜14はreservedとしてpacketを保持し、表示上は`Unknown`へ正規化します。
- packet queue overflow時はdrop countをconsoleへ表示します。永続logはGround Stationの責務外です。
- RSSI欠落時もXOR検証済みapplication packetを破棄せず、RSSI unavailableとして表示します。
- LEDは受信したMission packet headerに基づく表示です。A0 statusの任意bitをactuator commandとして使用しません。
- E220の正確なmodule型番、fixed prefixを含むGround UART raw境界、屋外range、A1〜A6の全header実機受信は未評価です。
