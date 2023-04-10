# Readme

このライブラリは、Cubic用の制御ライブラリです。

Cubic 2系に依存しています。

## About

`Cubic_controller::Velocity_PID` により速度制御を行います。`target`は、目標速度[rad/s]です。

`Cubic_controller::Position_PID` により位置制御を行います。`target`は、目標位置（角度[rad]）です。

それぞれ、内部で`DC_motor::put()`しています。

## Usage

初めに、各クラスのオブジェクト（例えば速度制御なら`Cubic_controller::Velocity_PID`）を、コンストラクタにより作成します。
各ループにおいて、`compute()`を実行します。
これにより、自動的に、適当なduty比が`DC_motor::put()`されます。
