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

timing診断だけは専用environmentを明示します。通常の`pio run`はproduction environmentだけをbuildします。

```sh
/home/hotaru/.platformio/penv/bin/pio run -e esp32doit-devkit-v1_lora_timing
/home/hotaru/.platformio/penv/bin/pio run -e esp32doit-devkit-v1_lora_timing \
  -t upload --upload-port /dev/ttyUSB<N>
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

uplinkはcapacity 1の単一boundary queueを所有し、一つのdownlink完了境界を一回のuplinkだけが消費します。起動時と任意のB1受信時にPeriodic streakを0へ戻し、B1自体は常に公開します。通常状態のA0〜A4は3件連続受信するまで非公開とし、3件目でPeriodic modeへ移行します。これによりB1を1件取りこぼしてもA0〜A4へ誤遷移しません。Periodic modeへの初期遷移は500 ms周期なら最大約1.5秒です。GroundTimeResponseはB1境界だけを受理し、その他のcommandは単一queueのB1またはA0〜A4へ揃えます。A5 Recovery、A6、B0は境界に使用しません。Recovery中の通常/local commandは2200 msでboundary timeoutとなりfail-closedにし、Emergencyだけは同timeout後にavailability fallbackを実行します。各iterationでは先にE220のAUX Highを成立させてから境界を待ち、UART TX commit直前にAUX Highと20 ms以内の境界ageを再確認します。AUX Lowまたは古い境界なら破棄して次の境界を待ちますが、最初のAUX readyからの2200 ms deadlineはinvalidated境界やEmergency preemptでもresetしません。telemetryが5秒途絶えた時点でもqueueとPeriodic streakをresetし、`micros()` wrap後の古い境界再利用と、受信断中のschedule変更をsafe扱いすることを防ぎます。

Actuator/Liftoff Emergencyはcapacity 2の専用FIFOで通常queueを迂回し、同種commandもcoalesceしません。満杯時は`emergency notification failed`を表示します。通常commandの境界/AUX待機中とUART TX commit直前にFIFOを確認しますが、commit後の一回のUART送信は中断しません。2200 ms以内に安全な境界を得られない場合だけEmergencyをAUX確認後に直接送るavailability fallbackとします。このfallbackはtelemetry継続中でもdownlinkと衝突し得ます。通常のtracked commandは境界timeout時にtransactionを解放してfail-closedにします。

LoRa command timingを測る場合は`esp32doit-devkit-v1_lora_timing` environmentを使用します。request、dequeue、境界kind/header/sequence/age/wait/fallback、AUX、UART write/flush、AUX High、B0 decode完了の時刻を`GROUND_LORA_TIMING` 1行形式で表示します。`aux_low_observed=0`のsampleはLow→High cycleによる送信完了を確認できていないため、完了時間の評価から分離します。通常environmentではこのlogを出しません。

## Test

```sh
sh test/run_host_tests.sh
/home/hotaru/.platformio/penv/bin/pio run
```

host testはMission/ComBoardとbyte-identicalな`testdata/99l_protocol_golden_vectors.txt`を読み、packet、bit境界、signed値、reserved/error、padding、checksum、uplink、transaction lifecycleを検証します。

## Hardware validation (2026-08-14)

Mission Board、Communication Board、Ground Boardの3基板を接続し、Groundは`/dev/ttyUSB0`の`Communication`モードで検証しました。

今回添付された`src.zip`の6 source fileは、修正前の`vault` checkoutとbyte単位で一致していました。修正前は通常commandとEmergencyの両方が入力後に次のdownlinkを最大2秒待つため、ComBoard側の周期とは別に位相依存の待ちを加えていました。

### 修正前・前回baseline

- **PASS**: production boot、decode task、command taskはpanic/resetなしで継続しました。
- **PASS**: read-only診断でE220に`C1 00 08`だけを送り、`C1 00 08 00 00 EC 81 04 C3 00 00`を読み戻しました。正式基準byteとは一致しますが、module型番は未確定のためfieldの意味はraw一致までの評価です。設定writeは行っていません。
- **PASS**: 130秒captureで203 packetを受信しました。内訳はA0=116、B0=3、B1=84、受信間隔は最小0.598 s/平均0.642 s/最大1.320 s、1秒超gap=1、5秒超gap=0です。RSSIは203/203 packetにあり、このcaptureでは最小/平均/最大すべて-107 dBmでした。decode errorとGround packet queue overflowはありませんでした。
- **PASS**: 最終productionの65秒同時captureで103 packet（A0=38、B1=63、B0=2）、1秒超gap 1、5秒超gap 0、RSSI missing 0、panic/reset loopなしを確認しました。`g 0x7F`と`le`はどちらもuplink送信後0.360秒で期待の終端B0を返し、pending/unmatched warningは0です。rawは`/tmp/99l_hwtest_20260814_G8Eh5f/final_production_65s_v5/`です。
- **PASS**: `g 0x7F` → Rejected/NotSupported、`g 0x02` → Rejected/InvalidState、`le` → Rejected/InvalidState、`ae` → Completed/None、GNSS/logging/recoveryの安全側local commandでGround→LoRa→ComBoard→CAN→Mission→CAN→ComBoard→LoRa→Groundを確認しました。未知generic commandの入力から終端B0までの最短実測latencyは0.515 sでした。
- **PASS**: 実機で使用したtransaction ID 4〜14はすべて0以外で、Acceptedと終端B0は同じIDで受信しました。終端B0でのID解放はhost testで検証済みです。
- **NOT_EVALUATED**: 実機でID wrap/exhaustionまで送るpending leak試験と、unknown/malformed B0で別IDを解放しないことは実施していません。`release`は必要なpending喪失がなかったため実行していません。
- **NOT_IMPLEMENTED / SPEC_GAP**: 最新VaultのUSB line protocol v1が定義する`@RX` / `@TX` / `@FRAG` / `@SYS`と`#`行は未実装です。今回は既存pretty-printをhost側でtimestamp付きcaptureして検証しました。

### 今回のscheduler / command timing

- **PASS**: 最大2秒の固定window待ちを削除し、B1でresetされる3件連続Periodic streak、単一boundary queue、20 ms fresh判定、2200 ms有限timeout、capacity 2 Emergency FIFO、UART TX commit直前pollを実装しました。
- **PASS**: `sh test/run_host_tests.sh`、production build、timing build、`git diff --check`を完走しました。
- **PASS（最終hardening前の参考値）**: B1混在35秒試験では`g 0x7F` 5回と`le` 5回の10/10が終端B0まで成功し、使用境界は全てB1、Emergency fallbackは0、AUX Low観測は10/10でした。入力から終端B0は`g 0x7F`が平均1081.980/最大1083.915 ms、`le`が平均1081.398/最大1082.885 msで、1.6秒以上のtail、CAN/LoRa/AUX error、queue dropは0でした。rawは`/tmp/99l_lora_scheduler_20260814T141137/final_mixed_b1_streak_35s/`です。
- **PASS（最終hardening前の参考値）**: 同種`le`を10 ms間隔で2件投入し、2/2が別transaction ID、別B1境界、別の終端B0で完了しました。同種coalesceは0でした。rawは`/tmp/99l_lora_scheduler_20260814T141137/final_emergency_fifo_pair/`です。
- **PASS（最終hardening前の参考値）**: productionの最終74.503秒captureでは150 packet（A0=146、B1=2、B0=2）、受信間隔は平均500.018 ms、p95 502.127 ms、p99 725.410 ms、最大727.584 ms、1秒超gap 0でした。`g 0x7F`は699.628 ms、`le`は701.370 msで終端B0まで完了しました。60.199秒のHWSTAT差分はCAN RX/TX=15345/1、LoRa TX/RX=121/2で、CAN/LoRa/AUX errorとqueue dropは0、B0優先によるperiodic missedは2でした。Missionのsafe outputs、runtime、encoder、CANは正常、Ground productionのtiming logは0でした。rawは`/tmp/99l_lora_scheduler_20260814T141137/final_production_75s_streak/`です。
- **PASS**: 最終hardeningを含むcurrent timing firmwareのB1混在13.252秒試験では28 packet（A0=10、B1=14、B0=4）を受信し、`g 0x7F` 2回と`le` 2回の4/4が期待した終端B0まで成功しました。4/4がB1境界、Emergency fallback 0、AUX Low観測4/4で、commit再確認によりinvalidated境界を1件安全に破棄しました。入力から終端B0は`g 0x7F`が最小1170.785/最大2169.401 ms、`le`が最小1169.352/最大1169.894 msでした。1.6秒以上のtailは`g 0x7F`の1件だけでabsolute deadline内に完了し、error/dropは0でした。rawは`/tmp/99l_lora_scheduler_20260814T141137/final_current_mixed_b1/`です。
- **PASS（全位相sweep前の参考値）**: current productionの69.500秒captureでは140 packet（A0=136、B1=2、B0=2）、受信間隔は平均500.000 ms、p95 502.279 ms、p99 749.474 ms、最大751.239 ms、1秒/5秒超gap 0でした。`g 0x7F`は926.277 msでRejected/NotSupported、`le`は927.211 msでRejected/InvalidStateの終端B0を返しました。60.198秒のHWSTAT差分はCAN RX/TX=15350/1、LoRa TX/RX=121/2で、CAN/LoRa/AUX errorとqueue dropは0、periodic missedは2でした。Missionのsafe outputs、runtime、encoder、CANは正常、Ground productionのtiming診断lineは0でした。rawは`/tmp/99l_lora_scheduler_20260814T141137/final_current_production_70s/`です。
- **PASS**: 最終current timing firmware（SHA256 `8582643ca6e6426d5e765d827a475b17a5d5e5b5c0efaa7dec36a72048ad90f8`）で、A0受信を基準に0〜450 msを50 ms刻み、各位相5回ずつ`g 0x7F`と`le`を送る100試行を実施しました。100/100が期待した終端B0まで成功し、timeout、late final、release、Emergency fallback、invalidated境界は0でした。入力から終端B0は`g 0x7F`が最小528.693/平均742.953/p95 967.395/最大968.323 ms、`le`が最小531.953/平均743.734/p95 966.374/最大968.250 msで、1秒および1.6秒以上のtailは0です。時刻応答を含むtiming sample 101/101でAUX Low→Highを観測し、AUX timeout、CAN/LoRa error、queue dropは0でした。rawとanalysisは`/tmp/99l_final_current_sweep_LpJ8fHJC/full_phase_100/`です。
- **PASS**: Groundを最終current production（SHA256 `f3e65fc16a9bfa73e965dd025555c4a4994734c93ca4d753aa1c236cc8d1f03d`）へ戻した69.501秒captureでは140 packet（A0=138、B0=2）、受信間隔は平均500.007 ms、p95 502.390 ms、p99 519.764 ms、最大520.596 ms、1秒/5秒超gap 0でした。`g 0x7F`は909.386 msでRejected/NotSupported、`le`は909.339 msでRejected/InvalidStateの終端B0を返しました。完全なHWSTAT同士の50.166秒差分はCAN RX/TX=12794/2、LoRa TX/RX=100/2で、CAN/LoRa/AUX errorとqueue dropは0、periodic missedは2でした。productionに`GROUND_LORA_TIMING`行はなく、panic/reset loopもありませんでした。Mission productionのsafe outputs、runtime、encoder、CAN起動は直前の同一試験captureで正常を確認しました。rawとanalysisは`/tmp/99l_final_current_sweep_LpJ8fHJC/final_production_70s/`です。
- **NOT_EVALUATED**: Emergency FIFO fullとsafe boundary timeout後のEmergency fallbackは未測定です。
- **NOT_EVALUATED**: A5 RecoveryではA5をuplink境界に使わず、通常/local commandをtimeoutでfail-closed、Emergencyだけを2200 ms後にavailability fallbackとする挙動は実機未評価です。
- **PENDING**: 残TODOとして、B1を2件連続で取りこぼした場合の識別と、B1/Periodic mode遷移をradio scheduleと明示的に同期するprotocolは未実装です。
- **PASS（統合前の参考値）**: A0単独124.659秒で251 packetを受信し、Ground受信間隔はp50 500.025 ms、p95 501.228 ms、p99 502.105 ms、最大519.931 ms（起動直後の最小155.903 msを含む）、1秒/5秒超gap 0でした。CAN/LoRa error、AUX timeout、queue drop、periodic missedは全て増加0です。
- **PASS（統合前の参考値）**: 時刻未同期のA0+B1混在124.257秒ではA0=125、B1=125でした。A0→B1は平均250.170 ms、B1→A0は平均749.881 ms、各header周期は約1秒で、古いB1 backlog、1秒/5秒超gap、CAN/LoRa error、queue dropはありませんでした。
- **PASS（統合前の参考値）**: A0受信を基準に0〜450 msを50 ms刻み、各位相5回ずつ`g 0x7F`と`le`を送る100試行は100/100で期待した終端B0を受信し、timeout/late final/releaseは0でした。入力→終端B0は`g 0x7F`が最小526.740/平均742.818/p95 966.976/最大967.438 ms、`le`が最小532.876/平均743.610/p95 966.603/最大968.389 msで、1秒および1.6秒以上のtailは0です。
- **PASS（統合前の参考値）**: Ground実機の境界ageは`g 0x7F`で最大11.369 ms、`le`で最大18.347 ms、Emergency fallbackは0でした。時刻応答は次のB1を974.661 ms待ち、境界age 44 usで元のrequest IDを送信しました。Ground uplink 101 sampleは全てAUX Low→Highを観測し、Ground packet drop、ComBoard CAN/LoRa error、AUX timeout、各queue dropは増加0です。
- **PASS（統合前の参考値）**: 3基板をproductionへ戻した最終73.998秒captureで149 packet（A0=145、B1=2、B0=2）を受信しました。全packet受信間隔は平均499.983 ms、p95 501.140 ms、最大749.540 ms、1秒/5秒超gap 0です。`g 0x7F`は692.940 msでRejected/NotSupported、`le`は692.220 msでRejected/InvalidStateの終端B0を返しました。ComBoardのCAN RX/TXは15349/2、LoRa TX/RXは120/2増加し、CAN/LoRa/AUX/queue drop errorは全て0、B0優先によるperiodic missedは2でした。Missionのsafe outputs、runtime、encoder、CanTaskも正常でした。

## Hardware assumptions / known limitations

- E220はUART 115200 bps、SF8/BW125、CH4、append-RSSIの既存設定を前提とします。
- A0 status bitとA6/B0/B1 layout、requested torque scaleはVaultの実装仮定台帳に記録した暫定値です。
- A0のfin/para mode 6〜14はreservedとしてpacketを保持し、表示上は`Unknown`へ正規化します。
- packet queue overflow時はdrop countをconsoleへ表示します。永続logはGround Stationの責務外です。
- RSSI欠落時もXOR検証済みapplication packetを破棄せず、RSSI unavailableとして表示します。
- LEDは受信したMission packet headerに基づく表示です。A0 statusの任意bitをactuator commandとして使用しません。
- E220の正確なmodule型番、fixed prefixを含むGround UART raw境界、屋外range、A1〜A6の全header実機受信は未評価です。
