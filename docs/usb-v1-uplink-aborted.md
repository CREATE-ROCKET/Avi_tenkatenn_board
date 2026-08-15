# USB v1 `UPLINK_ABORTED`

地上受信基板がE220 UARTへ1 byteも書き込む前に送信を中止した場合、次を出力する。

```text
@SYS usb_v=1 board_ms=<u32> event=UPLINK_ABORTED kind=<0..4> id=<1..255> command=0xHH error=<AUX_TIMEOUT|BOUNDARY_TIMEOUT>
```

## 契約

- `@TX`はE220 UARTへの書込みを実際に試行した場合だけ出力する。
- `UPLINK_ABORTED`はUART writeのcommit pointより前だけで出力する。
- 予約済みtransaction IDは、`UPLINK_ABORTED`を出力する前に解放する。
- firmwareは自動再送しない。GUIは当該操作を`BOARD_TX_FAILED`で終端する。
- `kind`、`id`、`command`は生成済みuplink frameと同じ値を使う。

## `error`

- `AUX_TIMEOUT`: UART write前にAUX Highを得られなかった。
- `BOUNDARY_TIMEOUT`: 通常commandで安全なdownlink境界を期限内に得られなかった。

Emergencyのboundary timeout後direct-send方針は変更しない。direct-send前にAUX timeoutした場合は`AUX_TIMEOUT`で中止する。
