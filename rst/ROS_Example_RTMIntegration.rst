



OpenHRP 環境の構築
^^^^^^^^^^^^^^^^^^

- 環境の確認

-- 自分のつかっているUbuntuのバージョンを調べて下さい．

::

  $ lsb_release -a

です．10.04か10.10であることが期待されています．


- ipv6 の設定の確認(10.04限定）


10.04の人は，
http://code.google.com/p/rtm-ros-robotics/wiki/RTM_Install 
にある通り，ipv6の設定によりlocalhostという名前解決ができないことがあるため，その場合，root権限で/etc/hostsの5行目をコメントアウトする．即ち，
::

  ::1     localhost ip6-localhost ip6-loopback   

を

::

  #::1     localhost ip6-localhost ip6-loopback  

と変更して保存する．Makefileでチェックしているので，変更していないとMakeが通りません．

- Java の確認


::

  $ rosdep install openhrp3
  $ java -version

としたときに、
::

  java version "1.6.0_24"
  Java(TM) SE Runtime Environment (build 1.6.0_24-b07)
  Java HotSpot(TM) 64-Bit Server VM (build 19.1-b02, mixed mode)

という風にJava(TM) SE が出ている必要があります．
この文字列が出てこない場合は、
::

  $ sudo update-java-alternatives -s java-6-sun

とする


- 旧バージョンOpenRTMの確認(インストールしたことがある人限定) 


::

  rtm-config --version

とうって
::

  0.4.2

と出た人は，古いOpenRTMが入っているので
::

  cd /usr/local/src/OpenRTM/OpenRTM-aist-0.4.2
  sudo make uninstall

としてください

- Eclipse環境のクリーンアップ(インストールしたことがある人限定) 

いままでEclipseを使ったことがある人は
::

  roscd openhrp3
  make eclipse-clean

として環境をクリーンにしてください．

OpenHRPのコンパイル
^^^^^^^^^^^^^^^^^^^

::

  rosmake rtmros_common

Eclipseの設定(RT System Editor編)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. <wiki:video url="http://www.youtube.com/watch?v=Majjva2YhX4" />

動画(youtube)
 http://www.youtube.com/watch?v=Majjva2YhX4


外部プラグインのインストール for RT System Editor 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

::

  rosrun openhrp3 eclipse.sh

で立ち上がるeclipseを用いる

 Help -> Install New Softare -> Add -> Location

に
(10.04ユーザのみ）
http://download.eclipse.org/modeling/emf/updates/releases/
を追加し、EMF SDK 2.5.0 を選択しインストール.

(10.04/10.10ユーザ共通)
http://download.eclipse.org/tools/gef/updates/releases/
を追加し、GEF SDK 3.6.2を選択しインストール．

 Help -> Install New Softare -> Add -> Archive

で，`<openhrp3>/build/rtmtools_eclipse.zip`
を選択しインストール．

<openhrp3>はrospack find openhrp3で表示されるディレクトリであり，例えば
~/prog/rtm-ros-robotics/rtmros_common/openhrp3になる．

ここまででRT System Editorが入る
::

  rosrun openhrp3 eclipse.sh

でEclipseを立ち上げて、
 Window -> Open Perspective -> Other 

で、RT System Editorが入っていることが確認できる

RT System Editor 動作確認
^^^^^^^^^^^^^^^^^^^^^^^^^

以前講義で試したサンプル
http://code.google.com/p/rtm-ros-robotics/wiki/RTM_Example
を試す.
::

  roslaunch openrtm MobileRobot_2DSimulator_Example.launch

http://code.google.com/p/rtm-ros-robotics/wiki/RTM_2DSimulator_Example
にあるように，「TkMobileRobotSimulator.py」が起動したら,「Create」ボタンを一回押す． 

他に
::

  roslaunch openrtm Slider_and_Motor_Example.launch
  roslaunch openrtm Seq_Example.launch

がある。

.. figure :: images/MobileRobot_2DSimulator_Example.png

.. figure :: images/Slider_and_Motor_Example.png

.. figure :: images/Seq_Example.png

4. Eclipseの設定(GXUI編) 
^^^^^^^^^^^^^^^^^^^^^^^^

外部プラグイン並びにのJava環境のインストール for GRXUI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

(10.04/10.10ユーザ共通)
Help -> Install New Softare -> Add -> Archive から

`<openhrp3>/build/java3declipse-20090302.zip`,

`<openhrp3>/build/grxui_eclipse.zip`,

`<hrpsys>/build/robothardware_eclipse.zip`

をそれぞれインストール

::

  rosrun openhrp3 eclipse.sh

でEclipseを立ち上げて、
 Window -> Open Perspective -> Other 

で、GRXUIが入っていることを確認する。

GRXUI 動作確認
^^^^^^^^^^^^^^^

.. <wiki:video url="http://www.youtube.com/watch?v=6wEH-41rw74" />

youtube
 http://www.youtube.com/watch?v=6wEH-41rw74

::

  roslaunch mrobot_ros_bridge mrobot_simulator.launch 

として、視点を下に移すと台車ロボットが見えればOK.wiimoteがないと言われる場合は，`aptittude install ros-diamondback-joystick-drivers`とするとよい．
::



Sample HRP4C
^^^^^^^^^^^^

roslaunch hrpsys hrp4c.launch

として，左上の→が描いてあるシミュレーション開始ボタンを押し， 中心に
  あるExecute scriptボタンをおして下図のようにロボットが歩けば成功です．
  このとき、右上にあるFPSのつまみの50を1～10程度にするとシミュレーショ
  ン速度が速くなります。

Execute scriptボタンの代わりに

roscd hrpsys/share/hrpsys/samples/HRP-4C;
rosrun hrpsys hrpsyspy ./HRP4C.py

とすることもできます。

.. image :: ../wiki/hrp4c_02.jpg

Sample PA10
^^^^^^^^^^^

roslaunch hrpsys pa10.launch
または
roslaunch openhrp3 pa10.launch

としてPA10が画面に表示され、左上のシミュレーション開始ボタンを押し動け
  ばOK

.. image :: ../wiki/roslaunch_openhrp3_pa10.jpg

Sample sample-vehicle
^^^^^^^^^^^^^^^^^^^^^

roslaunch openhrp3 sample-vehicle.launch

左上のシミュレーション開始ボタンを押し動けばOK.

.. image :: ../wiki/roslaunch_openhrp3_sample_vehicle.jpg

Sample Humanoid Robot
^^^^^^^^^^^^^^^^^^^^^

roslaunch openhrp3 samplerobot-walk.launch

左上のシミュレーション開始ボタンを押してしばらく待つとロボットが歩きだ
  す。

.. image :: ../wiki/sample_robot_walk.jpg

roslaunch openhrp3 samplerobot-pickupbox.launch

左上のシミュレーション開始ボタンを押してしばらく待つとロボットが箱を持
  ち上げる。

.. image :: ../wiki/sample_robot_pickupbox.jpg

roslaunch openhrp3 samplerobot-inhouse.launch

左上のシミュレーション開始ボタンを押してしばらく待つとロボットが机の上
  の箱をつかみにいく。 

.. image :: ../wiki/sample_robot_inhouse.jpg



トラブルシューティング
^^^^^^^^^^^^^^^^^^^^^^

- Check OmniORB bug


$ rosrun openrtm rtm-naming-restart

and start following command from different terminal

$ rosrun openhrp3 openhrp-aist-dynamics-simulator -ORBInitRef
NameService=corbaloc:iiop:localhost:2809/NameService

If you see "ready", than it ok, if you see
"IDL:omg.org/CORBA/TRANSIENT:1.0", that's would be omniorb bug in
ubuntu package.

Please comment out

::1     localhost ip6-localhost ip6-loopback

line from /etc/hosts file
Check SVN version

make sure that you have downloaded latest version of repository

$ roscd rtmros_common; svn up

if you have find any update, then rosmake hrpsys hrpsys_ros_bridge
again

- Check Java version


OpenHRP3 assume SUN version of java and not GNU or other
implementation.

$ java -version
  java version "1.6.0_26"
 Java(TM) SE Runtime Environment (build 1.6.0_26-b03)
 Java HotSpot(TM) 64-Bit Server VM (build 20.1-b02, mixed mode)

if it is not sun java, rosdep install openhrp3 or `sudo
update-java-alternatives -s java-6-sun`

- Check OpenHRP simulation


Please make sure that OpenHRP simulation works.

$ rosrun openhrp3 grxui.sh

select "GrxUI -> Load Project" menu and select FallingBoxes?.xml Then
press "Start Simulation" button, to see if the 3 yellow boxes falling
down.

Then select SampleRobot?_inHouse.xml file and press "Start Simulation"
button to see that robot start walking.

If this not working, you may fail to install eclipse plugin

rm -fr ~/.eclipse
roscd openhrp3; rm -fr workspace

and then setup eclipse to install the plugins again.

- Check HiroNX collada file

Make sure that you have downloaded HiroNX collada file

 rosls collada_robots/data/robots/kawada-hironx.dae

if not, roscd collada_robots; rm installed; make
Check OpenHRP Collada support

Make sure that your openrhp3 support to load collada files

$ roslaunch openhrp3 grxui.launch

and right click Model menu on the left "Item View" and select "load",

Select kawada-hironx.dae under
jsk-ros-pkg/openrave_planning/collada_robots/data/robots directory

Select *.dae from buttom left menu, that currently shown as *.wrl, and
choose kawada-hironx.dae model, then confurm if you can see kawada
robot model on the screen. 
