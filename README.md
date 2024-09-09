# Bala2 micro-ROS controller

M5stack Bala2 をmicro-ROS with FreeRTOS で動かすサンプルコードです．
このコードをベースにして作り変えて遊んでみてください．


## How to use
### Quick start
1. Arduino IDEで各種環境構築を行う．
	- ライブラリで"ESP32"をインストールする．なお，バージョンは3.0.0未満しか対応していないので，2.0.17などにしておく．

1. "m5bala2_uros.ino"をArduino IDEで開き，bala2に書き込む．


### 制御方法
サンプルコードでは，本体の傾斜角を表す`incli_angle`と旋回角度`steer_angle`の値を変えると，それぞれによって直進，旋回ができるようになる．

`incli_angle`と`steer_angle`をROS2 topic通信で受け取るサブスクライバーは実装済みなので，他のデバイスでそれらの値をpublishして遠隔操作できるようにしてください．
