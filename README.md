# ROSExt Plugin

## ClockPublisherItem
/clockをpublish. choreonoid1.8がreleaseされたらhttps://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/WorldROSItem.h を使いたい

## CameraPublisherItem
camera画像をpublish. https://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/BodyROSItem.h との差異は、欲しいcamera画像だけをpublishする点と、camera_infoもpublishする点.

## DepthCameraPublisherItem
depth_cameraのimage,depth,pointcloudをpublish. https://github.com/choreonoid/choreonoid_ros/blob/master/src/plugin/BodyROSItem.h との差異は、欲しいcameraだけをpublishする点と、camera_infoとdepth画像もpublishする点.

## OdometryCameraPublisherItem
cameraの現在地と速度をnav_msgs/Odometryでpublish. (Ground Truthになってしまう)

## CraneItem
ロボットを吊るすクレーン. `<name>/Lift`に`std_srvs/SetBool`サービスを送るとクレーンをオン, オフできる.

## SimulatorWorldResetItem
シミュレータ内の全Bodyを初期状態にリセットする. `<name>/Reset`に`std_srvs/Trigger`サービスを送るとリセットできる.`SimulatorItem::startSimulation`との差異は、時計がリセットされない点.

## PositionDraggerItem
(ROSとは関係ない). GUIからマウスで位置姿勢を指定すると、リンクに力を作用させてその位置に固定できる. ツールバーのボタンからオンオフを切り替えられる。

# SimpleController

## SpringDamplerController
関節をSpringDamper化するSimpleController
