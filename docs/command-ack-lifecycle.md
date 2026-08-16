# Mission generic command ACK contract

Ground BoardはLoRa transportとtransaction IDの所有者であり、Mission generic commandのoperator-facing ACK timeoutはPC GUIが所有する。

## ACK

Mission generic commandでは、同一transaction ID / command codeの`CommandResult`について

```text
phase=Accepted
reason=None
```

をACKとする。独立したACK packetは追加しない。

Ground Boardが出力する`@TX ok=1`はE220へのuplink送信完了を表すだけで、Mission BoardのACKではない。受信したB0は従来どおり`@RX`として即時にUSBへ公開する。

## 3秒timeout

`Avi_99L_GroundFirmware`は対応する`@TX ok=1`受信後3000 ms以内に`Accepted / None`を観測できなければ`COMMAND ACK TIMEOUT`を表示する。

このtimeoutはGround Board firmwareでは生成しない。USB protocolへtimeout専用recordを追加せず、session logの`@TX`と`@RX B0`からGUIが判定する。

理由は、AcceptedだけがLoRa downlinkで欠落してterminal B0が後着する可能性があるためである。Ground Boardが3秒でtransaction IDを解放すると、遅延B0と再利用IDが衝突し得る。

したがってGround BoardはACK timeoutを理由に次を行わない。

- transaction IDの自動解放
- 同一commandの自動再送
- pending entryの上書き
- `@TX ok=1`をACKへ読み替えること

terminal `Completed / Rejected / Failed`を受信した場合の既存transaction解放規則は変更しない。B0を失ったtransactionはoperatorが状態を確認した後、既存`release <transaction_id>`で明示解放できる。

## 対象外

`ActuatorEmergencyStop` / `LiftoffDetectionEmergencyStop`は直接terminal resultを返し得るため、generic commandのAccepted必須規則を適用しない。

ComBoard local commandもownerが異なり、直接Completedを返すcommandがあるため対象外とする。

## Vault

正本のoperator-facing lifecycleは`Natsu-B/Vault`の`CREATE/99L Ground Station/03_Command ACK lifecycle.md`を参照する。
