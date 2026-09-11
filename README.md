# Robot Triathlon - H8/3664F → RP2040 移植プロジェクト

室蘭工業大学のロボットトライアスロン用標準キットプログラムを、H8/3664FからRP2040（Raspberry Pi Pico）に移植したプロジェクトです。

オリジナルは http://kamuy.elec.muroran-it.ac.jp/robocon2/making/2004/techknow/program.html （Wayback Machine）で公開されていたものです。

## ディレクトリ構成

```
├── H8_3664F/          # オリジナル（H8/3664F用）
│   ├── core.mar       # H8アセンブラ（割込み・ハードウェア初期化）
│   ├── robot.h        # Cライブラリ（制御アルゴリズム・数学関数）
│   └── robot.c        # ユーザープログラムテンプレート
├── RP2040/            # 移植版（RP2040/Pico SDK用）
│   ├── pin_config.h   # GPIOピンマッピング定義
│   ├── hw.c           # ハードウェア抽象層（core.mar代替）
│   ├── robot.h        # 制御ライブラリ（標準math.h使用）
│   ├── robot.c        # ユーザープログラムテンプレート
│   ├── CMakeLists.txt # Pico SDK用ビルド設定
│   └── README.md      # 移植版の詳細ドキュメント
└── .gitignore
```

## H8 → RP2040 の主な変更点

1. **アセンブラ → C**: `core.mar`の割込み処理・ハードウェア初期化をすべてPico SDKのC APIで実装
2. **カスタム数学関数 → 標準ライブラリ**: `sin`/`cos`/`sqrt`を標準`math.h`の`sinf`/`cosf`/`sqrtf`に置き換え
3. **H8レジスタ → Pico SDK API**: `IO.PDR5.BIT.Bx` → `gpio_put`/`gpio_get`、`TW.GRB`/`TW.GRC` → `pwm_set_chan_level`
4. **ADC**: RP2040の12bit ADCを右シフト2で10bit相当に変換（`THRESHOLD`等の定数をそのまま流用）
5. **PWM**: H8と同じ20kHz・duty 0-799を再現（分周7.8125、wrap=799）
6. **制御周期**: 10.24msの繰り返しタイマーでH8と同等の制御周期を実現

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

- [Pico SDK](https://github.com/raspberrypi/pico-sdk)
- arm-none-eabi-gcc
- CMake + Ninja（または Make）

### 手順

```bash
# pico_sdk_import.cmake をPico SDKからコピー
cp /path/to/pico-sdk/external/pico_sdk_import.cmake RP2040/

# ビルド
cd RP2040
mkdir build && cd build
cmake ..
make
```

### 書き込み

**UF2方式（簡単）:**
1. PicoのBOOTSELボタンを押しながらUSB接続
2. マスストレージデバイスとして認識される
3. `robot_triathlon.uf2` をドラッグ&ドロップ
4. 自動再起動してプログラム実行

**SWD方式（デバッグ可能）:**
```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
  -c "adapter speed 5000; program robot_triathlon.elf verify reset exit"
```

## 注意事項

- センサ類は5V→3.3Vレベル変換が必要
- モータドライバへの入力も3.3V出力となるため、5V駆動のドライバを使用する場合はレベル変換を検討
- DIPスイッチはアクティブLow（プルアップ付き）として読み込む

## ライセンス

オリジナルプログラム by YOU-YOU ROBOTICS (2002-2004)
