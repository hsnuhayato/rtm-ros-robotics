*summary RTM-ROS Integration*

.. <wiki:toc max_depth="2" />

====
概要
====

====================
1.OpenHRP 環境の構築
====================

----------
環境の確認
----------

- 自分のつかっているUbuntuのバージョンを調べて下さい．

::

  $ lsb_release -a

です．10.04か10.10であることが期待されています．

-----------------------------
ipv6 の設定の確認(10.04限定）
-----------------------------

10.04の人は，
http://code.google.com/p/rtm-ros-robotics/wiki/RTM_Install 
にある通り，ipv6の設定によりlocalhostという名前解決ができないことがあるため，その場合，root権限で/etc/hostsの5行目をコメントアウトする．即ち，
::

  ::1     localhost ip6-localhost ip6-loopback   

を

::

  #::1     localhost ip6-localhost ip6-loopback  

と変更して保存する．Makefileでチェックしているので，変更していないとMakeが通りません．

-----------
Java の確認
-----------

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


------------------------------------------------------------
旧バージョンOpenRTMの確認(インストールしたことがある人限定) 
------------------------------------------------------------

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

-------------------------------------------------------------
Eclipse環境のクリーンアップ(インストールしたことがある人限定) 
-------------------------------------------------------------

いままでEclipseを使ったことがある人は
::

  roscd openhrp3
  make eclipse-clean

として環境をクリーンにしてください．

======================
2. OpenHRPのコンパイル
======================

::

  rosmake rtmros_common

====================================
3. Eclipseの設定(RT System Editor編)
====================================

.. <wiki:video url="http://www.youtube.com/watch?v=Majjva2YhX4" />

動画(youtube)
 http://www.youtube.com/watch?v=Majjva2YhX4

--------------------------------------------------
外部プラグインのインストール for RT System Editor 
--------------------------------------------------

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

-------------------------
RT System Editor 動作確認
-------------------------

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

.. image :: MobileRobot_2DSimulator_Example.png

.. image :: Slider_and_Motor_Example.png

.. image :: Seq_Example.png

=========================
4. Eclipseの設定(GXUI編) 
=========================

------------------------------------------------------
外部プラグイン並びにのJava環境のインストール for GRXUI
------------------------------------------------------

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

==================
5. GRXUI 動作確認 
==================

.. <wiki:video url="http://www.youtube.com/watch?v=6wEH-41rw74" />

youtube
 http://www.youtube.com/watch?v=6wEH-41rw74

::

  roslaunch mrobot_ros_bridge mrobot_simulator.launch 

として、視点を下に移すと台車ロボットが見えればOK.wiimoteがないと言われる場合は，`aptittude install ros-diamondback-joystick-drivers`とするとよい．
::

  roslaunch hrpsys pa10.launch
  または、
  roslaunch openhrp3 pa10.launch


としてPA10が画面に表示され動けばOK

.. image :: http://rtm-ros-robotics.googlecode.com/svn/wiki/roslaunch_openhrp3_pa10.jpg

あるいは，
::

  roslaunch hrpsys hrp4c.launch

として，左上の→が描いてあるシミュレーション開始ボタンを押し，
中心にあるExecute scriptボタンをおしてロボットが歩けば成功です．

=========================
6. トラブルシューティング
=========================

------------------------------
モデルが表示されないときは... 
------------------------------

- "LoadProject"で"FallingBoxes"を選択しても3DViewにモデルがでない。

::

  roscd openhrp3; bin/openhrp-model-loader 

として
::

  ready

がでているか確認。でていない場合は、
::

  $ sudo /etc/init.d/omniorb4-nameserver stop
  $ pkill -KILL omniNames
  $ rtm-naming

とする．10.04の人は/etc/hostsの::1から始まる行がコメントアウトされているか再確認．

- "FallingBoxes"のシミュレーションは表示されるけど、"SampleRobot_inHouse.xml"をロードしてもロボットが表示されない場合、

::

  roscd openhrp3; make eclipse-clean; make wipe; roscd hrpsys; make wipe; rosmake

としてクリーンな環境をつくり必要なプラグインをインストール
