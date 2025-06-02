これは何？
===========

HSR-BのarduinoをUSB-IOとして使うためのarduinoのsketchと通信スクリプト.
HSR-B Phase5以降専用．

Subscribe
----------
### 全フェーズ対応

* command\_suction(std\_msgs/Bool): 吸引ポンプの制御(true: 吸引, false: ストップ)
* command\_status\_led\_rgb(std\_msgs/ColorRGBA): ロボットの状態表示LED(rgb値は[0.0, 1.0]の範囲で指定可能)
* command\_drive\_power(std\_msgs/Bool): 駆動系出力状態(true: On, false: off)
* command\_hand\_led(std\_msgs/Bool): ハンドのLEDの制御(true: 点灯, false: 消灯)
* command\_head\_led(std\_msgs/Bool): 頭部のLEDの制御(true: 点灯, false: 消灯)

Publish
--------

* base\_magnetic\_sensor\_1(std\_msgs/Bool): 磁気センサ1ラッチ状態(true: On, false: off)
* base\_magnetic\_sensor\_2(std\_msgs/Bool): 磁気センサ2ラッチ状態(true: On, false: off)
* runstop\_button(std\_msgs/Bool): キノコスイッチ状態(true: 停止, false: 動作)
* pressure\_sensor(std\_msgs/Bool): 吸引状態(true: センサ反応, false: センサ非反応)
* base\_f\_bumper\_sensor(std\_msgs/Bool): フロントバンパセンサ(true: センサ反応, false: センサ非反応)
* base\_r\_bumper\_sensor(std\_msgs/Bool): リアバンパセンサ(true: センサ反応, false: センサ非反応)

全て約10Hzで出力

プログラムの書き込み
============

GUIからの書き込み
------------------

1. arduinoをロボット上でインストール
2. arduinoをlinux上で起動
3. arduinoからsketchを開く sketches/hsrb_usbio_3_serial.ino
4. 検証・コンパイルでコンパイル
5. ツール->シリアルポートから適切なシリアルポートを選択(ttyUSB0 or ttyUSB1)
6. ツール->マイコンバードからArduino UNOを選択
7. マイコンボードに書き込むで書き込み
