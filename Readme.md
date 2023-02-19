# Readme

このライブラリは、Cubic用の制御ライブラリです。

Cubic 2系に依存しています。

## Usage

`Cubic_controller::Velocity_PID` により速度制御を行います。`target`は、目標速度[rad/s]です。

`Cubic_controller::Position_PID` により位置制御を行います。`target`は、目標位置です。（コンストラクタ呼び出し時を0とした角度[rad]）

それぞれ、内部で`DC_motor::put()`しています。

## Update history

