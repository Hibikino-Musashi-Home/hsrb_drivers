Overview
++++++++

提供機能
--------

- Congatec社製CPUを制御する
  GPIOの入出力管理を行うhardware Interface
  pub/subを行うコントローラを挿すことでpub/subができるようになる

ROS Interface
++++++++++++++

Nodes
-----

- **cgos_gpio** GPIOノード

Subscribed Topics
^^^^^^^^^^^^^^^^^

GPOのトピックを購読する。
トピック名は、パラメータで指定する。

- **/cgos_gpio/output0** GPOの出力値

Published Topics
^^^^^^^^^^^^^^^^

GPIのトピックを発行する。
トピック名は、パラメータで指定する。

- **/cgos_gpio/input0** GPIの入力値

Parameter
^^^^^^^^^

- **~gpio** (gpio:
               input0:
                 pin: 0
                 direction: in
               output0:
                 pin: 4
                 direction: out) GPIOの設定
	                         項目名：sub/pubの名称となる
	                         pin：使用するGPIOのPIN番号
	                         direction：入力または出力
	                         入力は、pinに0〜3を設定する
	                         出力は、pinに4〜7を設定する

Internal
++++++++

.. ifconfig:: internal
