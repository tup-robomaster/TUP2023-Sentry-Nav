# TUP2023-Sentry-Nav
## 1.通讯协议
### TX:
#### 1.AutoAim
|Byte|Data|
|-|-|
|0|0xA5|
|1|mode|
|2|CRC8|
|3-6|Pitch|
|7-10|Yaw|
|11-14|Distance|
|15|isSwitched|
|16|isFindTarget|
|17|isSpinning|
|18|isMiddle|
|19-61|Empty|
|62-63|CRC16|
#### 2.Nav
|Byte|Data|
|-|-|
|0|0xB5|
|1|mode|
|2|CRC8|
|3-14|Linear Twist(XYZ)|
|15-26|Angular Twist(XYZ)|
|27-61|Empty|
|62-63|CRC16|

#### 3.Decision
|Byte|Data|
|-|-|
|0|0xC5|
|1|mode|
|2|CRC8|
|3-6|Theta Gimbal|
|7-10|Theta Chassis|
|27-61|Empty|
|62-63|CRC16|