# Robot Triathlon - RP2040 Port

H8/3664F用ロボットトライアスロン標準キットプログラムのRP2040（Raspberry Pi Pico）移植版

## ファイル構成

| ファイル | 説明 |
|---------|------|
| `pin_config.h` | RP2040 GPIOピンマッピング定義 |
| `hw.c` | ハードウェア抽象層（H8アセンブラ core.mar の代替） |
| `robot.h` | 制御ライブラリ（H8版 robot.h の代替、標準math.h使用） |
| `robot.c` | ユーザープログラムテンプレート |
| `CMakeLists.txt` | Pico SDK用ビルド設定 |

## H8との主な変更点

1. **アセンブラ → C**: core.marの割込み処理・ハードウェア初期化をすべてCで実装
2. **カスタム数学関数 → 標準ライブラリ**: sin/cos/sqrtを標準math.hのsinf/cosf/sqrtfに置き換え
3. **H8レジスタアクセス → Pico SDK API**: IO.PDR5.BIT.Bx → gpio_put/get、TW.GRB/GRC → pwm_set_chan_level
4. **ADC 10bit→12bit**: RP2040の12bit ADCを右シフト2で10bit相当に変換（THRESHOLD等の定数そのまま利用可）
5. **PWM**: H8と同じ20kHz・duty 0-799を再現

## ピンマッピング

| 機能 | H8/3664F | RP2040 GPIO |
|------|----------|-------------|
| ライントレースCLK | P10 | GPIO0 |
| 右エンコーダA | P14/IRQ0 | GPIO1 |
| 左エンコーダA | P15/IRQ1 | GPIO2 |
| 右ライン検出 | P16/IRQ2 | GPIO3 |
| 左ライン検出 | P17/IRQ3 | GPIO4 |
| DIP SW0 | P50 | GPIO5 |
| DIP SW1 | P51 | GPIO6 |
| LED0 | P52 | GPIO7 |
| LED1 | P53 | GPIO8 |
| 右モータ前進 | P54 | GPIO9 |
| 右モータ後退 | P55 | GPIO10 |
| 左モータ前進 | P56 | GPIO11 |
| 左モータ後退 | P57 | GPIO12 |
| ラインセンサEN | P74 | GPIO13 |
| 測距センサEN | P75 | GPIO14 |
| ユーザー入力 | P76 | GPIO15 |
| 右エンコーダB | P80 | GPIO16 |
| 左エンコーダB | P81 | GPIO17 |
| 右モータPWM | P82 | GPIO18 (PWM1A) |
| 左モータPWM | P83 | GPIO19 (PWM1B) |
| 追加モータPWM | P84 | GPIO20 (PWM2A) |
| ユーザー出力0 | P85 | GPIO21 |
| ユーザー出力1 | P86 | GPIO22 |
| 測距センサADC | PB0/AN0 | GPIO26 (ADC0) |
| ラインGND ADC | PB1/AN1 | GPIO27 (ADC1) |
| ライントレースADC | PB2/AN2 | GPIO28 (ADC2) |
| ユーザーADC | PB3/AN3 | GPIO29 (ADC3) |

## ビルド方法

### 前提条件
- Pico SDK (https://github.com/raspberrypi/pico-sdk)
- arm-none-eabi-gcc
- CMake + Ninja
- `pico_sdk_import.cmake` をこのディレクトリにコピー

### ビルド手順

```bash
# pico_sdk_import.cmakeをPico SDKからコピー
cp /path/to/pico-sdk/external/pico_sdk_import.cmake .

# ビルド
mkdir build && cd build
cmake ..
make
```

### 書き込み方法

#### UF2方式（簡単）
1. PicoのBOOTSELボタンを押しながらUSB接続
2. マスストレージデバイスとして認識される
3. `robot_triathlon.uf2` をドラッグ&ドロップ
4. 自動再起動してプログラム実行

#### SWD方式（デバッグ可能）
```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 5000; program robot_triathlon.elf verify reset exit"
```

## 注意事項

- センサ類は5V→3.3Vレベル変換が必要
- モータドライバへの入力も3.3V出力となるため、5V駆動のドライバを使用する場合はレベル変換を検討
- DIPスイッチはアクティブLow（プルアップ付き）として読み込む
