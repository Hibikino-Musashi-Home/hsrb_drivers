Overview
++++++++

提供機能
--------

- 電源ECUと通信し、バッテリ情報、センサ情報、診断情報を取得する。
- コマンドを送信し、電源ECUを制御する

ROS Interface
++++++++++++++

Nodes
-----

- **power_ecu** 電源ECU通信ノード

Subscribed Topics
^^^^^^^^^^^^^^^^^

- **command_status_led** (:ros:msg:`std_msgs/ColorRGBA`) 多用途LEDの点灯／点滅や色を指定する

- **motor_enable** (:ros:msg:`std_msgs/Bool`) 駆動系の電源を制御する

- **pump_enable** (:ros:msg:`std_msgs/Bool`) ポンプのスイッチを制御する

- **solsw_enable** (:ros:msg:`std_msgs/Bool`) ソレノイドスイッチを制御する


Published Topics
^^^^^^^^^^^^^^^^

- **diagnostics** (:ros:msg:`diagnostic_msgs/DiagnosticArray`) 電源ECUとバッテリの診断情報

- **base_b_bumper_sensor** (:ros:msg:`std_msgs/Bool`) バンパセンサ1の状態

- **base_f_bumper_sensor** (:ros:msg:`std_msgs/Bool`) バンパセンサ2の状態

- **base_magnetic_sensor_1** (:ros:msg:`std_msgs/Bool`) 近接センサ1ラッチ状態

- **base_magnetic_sensor_2** (:ros:msg:`std_msgs/Bool`) 近接センサ2ラッチ状態

- **base_magnetic_sensor_3** (:ros:msg:`std_msgs/Bool`) 近接センサ3ラッチ状態

- **base_magnetic_sensor_4** (:ros:msg:`std_msgs/Bool`) 近接センサ4ラッチ状態

- **base_magnetic_sensor_5** (:ros:msg:`std_msgs/Bool`) 近接センサ5ラッチ状態

- **bat_stat** (:ros:msg:`std_msgs/Bool`) バッテリ充電状態

- **battery_state** (:ros:msg:`tmc_msgs/BatteryState`) バッテリ情報

- **imu/data** (:ros:msg:`sensor_msgs/Imu`) ジャイロ、加速度センサ

- **pressure_sensor** (:ros:msg:`std_msgs/Bool`) pump電圧

- **sanyo_battery_info** (:ros:msg:`tmc_sanyo_battery/SanyoBatteryInfo`) バッテリ情報

- **sw_drv** (:ros:msg:`std_msgs/Bool`) 駆動SW

- **sw_kinoko** (:ros:msg:`std_msgs/Bool`) 有線緊急停止SW

- **sw_latch** (:ros:msg:`std_msgs/Bool`) ラッチ解除SW

- **sw_pwr** (:ros:msg:`std_msgs/Bool`) 電源(プリウスSW)

- **sw_w_sel** (:ros:msg:`std_msgs/Bool`) 無線切り替えSW

- **sw_w_stop** (:ros:msg:`std_msgs/Bool`) 無線緊急停止SW

- **charger_state** (:ros:msg:`std_msgs/UInt8`) 自動充電ステータス


Services
^^^^^^^^

- **imu** (:ros:srv:`std_srvs/Empty`) ジャイロ、加速度センサをリセットする

- **res_g** (:ros:srv:`std_srvs/Trigger`) クオタニオンをリセットする

- **res_q** (:ros:srv:`std_srvs/Trigger`) ジャイロのオフセットをリセットする

- **mute** (:ros:srv:`std_srvs/SetBool`) オーディオアンプの音声出力を制御する。(true:mute、false:mute解除)

- **pdcpu** (:ros:srv:`std_srvs/Trigger`) CPUをシャットダウンする

- **pdgpu** (:ros:srv:`std_srvs/Trigger`) GPUをシャットダウンする

- **pdex1** (:ros:srv:`std_srvs/Trigger`) 外部CPUをシャットダウンする

- **undck** (:ros:srv:`std_srvs/Trigger`) アンドック指令を発行する

- **enable_12vu** (:ros:srv:`std_srvs/SetBool`) 12V USBスイッチを制御する。(true:スイッチON、false:スイッチOFF)

- **enable_5vd3** (:ros:srv:`std_srvs/SetBool`) 5Vd3スイッチを制御する。(true:スイッチON、false:スイッチOFF)

- **enable_5vd4** (:ros:srv:`std_srvs/SetBool`) 5Vd4スイッチを制御する。(true:スイッチON、false:スイッチOFF)

- **enable_5vd5** (:ros:srv:`std_srvs/SetBool`) 5Vd5スイッチを制御する。(true:スイッチON、false:スイッチOFF)


Parameter
^^^^^^^^^

- **port_name** (string: /dev/ttyCTI3) 電源ECUと通信するシリアルポートの名前

- **port_receive_timeout_ms** (double: 0.5) シリアルポートリード時のタイムアウト時間 [ms]

- **imu_frame** (string: imu_frame) IMUのフレーム名

- **communication_error_threshold** (double: 0.01) 通信エラー閾値

- **battery_warning_threshold** (double: 50.0) バッテリー残量Warning閾値 [%]

- **battery_error_threshold** (double: 20.0) バッテリー残量エラー閾値 [%]

- **pump_voltage** (double: 8000.0) pump電圧しきい値 [mV]

- **config_file** 設定ファイル


指定可能な項目一覧

tmc_digital_input_controller:
  type: tmc_realtime_controllers/DigitalInputController
  publish_rate: 10

tmc_digital_output_controller:
  type: tmc_realtime_controllers/DigitalOutputController

tmc_sanyo_battery_controller:
  type: tmc_sanyo_battery/SanyoBatteryController
  publish_rate: 1

tmc_diagnostic_controller:
  type: tmc_realtime_controllers/DiagnosticController
  publish_rate: 1

tmc_empty_command_controller:
  type: tmc_realtime_controllers/EmptyCommandController
  service_time_out: 1000

tmc_set_bool_command_controller:
  type: tmc_realtime_controllers/SetBoolCommandController
  service_time_out: 1000

tmc_trigger_command_controller:
  type: tmc_realtime_controllers/TriggerCommandController
  service_time_out: 1000

imu_sensor_controller:
  type: imu_sensor_controller/ImuSensorController
  publish_rate: 100

tmc_color_command_controller:
  type: tmc_realtime_controllers/ColorCommandController
  command_status_led:
    timeout: 1.0
    timeout_color: [1.0, 1.0, 1.0]

charger_state_controller:
  type: hsrb_power_ecu/ChargerStateController
  publish_rate: 2


Hardware Interface
^^^^^^^^^^^^^^^^^^

以下のhardware_interface、ros_controllerを使用する。

===============================================  =================================================
hardware_interface                               ros_controller
===============================================  =================================================
tmc_hardware_interface::DiagnosticInterface      tmc_realtime_controllers/DiagnosticController
tmc_hardware_interface::DigitalInputInterface    tmc_realtime_controllers/DigitalInputController
tmc_hardware_interface::DigitalOutputInterface   tmc_realtime_controllers/DigitalOutputController
tmc_sanyo_battery::SanyoBatteryInterface         tmc_sanyo_battery/SanyoBatteryController
tmc_hardware_interface::EmptyCommandInterface    tmc_realtime_controllers/EmptyCommandController
tmc_hardware_interface::TriggerCommandInterface  tmc_realtime_controllers/TriggerCommandController
tmc_hardware_interface::SetBoolCommandInterface  tmc_realtime_controllers/SetBoolCommandController
hardware_interface::ImuSensorInterface           imu_sensor_controller/ImuSensorController
tmc_hardware_interface::ColorCommandInterface    tmc_realtime_controllers/ColorCommandController
hsrb_power_ecu::ChargerStateInterface            hsrb_power_ecu/ChargerStateController
===============================================  =================================================


Internal
++++++++

.. ifconfig:: internal

   振る舞い:
     * ros_controlのhardware_interface::RobotHWを継承したクラスを実装。
     * Nodeを起動すると、電源ECUに対し時計合わせ、MUTE解除、定期送信スタートのコマンドを送信する.
     * 電源EUCに対し、定周期でハートビートコマンドを送信する.
     * 購読したTopicsの内容に応じたコマンドを電源ECUへ送信する.
     * コマンドの実行結果が取得できない場合やNACKを受信した場合は、ROS_ERRORを出力する
     * yamlファイルから各Topicsの発行周期を変更できる.
