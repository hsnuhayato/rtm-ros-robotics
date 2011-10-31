*summary VPython環境の使い方*

============================
ダウンロード(以下のいずれか)
============================

rtm-ros-roboticsリポジトリをsvn checkoutしている場合は，
rtm-ros-roboticsディレクトリで，
::

  $ svn up


rosinstallを使っている場合は，
::

  $ rosinstall ~/prog/rtm-ros-robotics /opt/ros/diamondback http://rtm-ros-robotics.googlecode.com/svn/trunk/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall


rosinstallを使っている場合にこのソースだけsvn coしたい場合は，
::

  $ svn co agentsystem_hironx_samples


====================
環境設定とコンパイル
====================
::

  $ export ROS_PACKAGE_PATH=<rtm-ros-robotics>/agentsystem_hironx_samples:$ROS_PACKAGE_PATH

<rtm-ros-robotics>はリポジトリをチェックアウトした場所.
.bashrcに書いておくとよい.
rosinstallを利用している場合は，setup.bashでROS_PACKAGE_PATHに追加されるので明示的に追加する必要はない． 
::

  $ roscd iv_plan/externals;make
  $ rosmake iv_plan


vpythonが依存するlibgtkglextmm，パーサが使っているplyあたりの
debパッケージは入っていない可能性がある．
以下のようなメッセージが表示された場合は，rosmake iv_plan --rosdep-installとして
依存するdebパッケージをインストールする．
::

  [ rosmake ] WARNING: Rosdep did not detect the following system dependencies as installed: Did not detect packages: ['libgtkglextmm-x11-1.2-0', 'libgtkglextmm-x11-1.2-dev']
   Consider using --rosdep-install option or `rosdep install iv_plan`



========
デモ実行
========
::

  $ roscd iv_plan/examples
  $ ipython pickbox.py
  putbox()
  手が届く位置に箱を置く
  pick()
  手が届かないとエラーになるので，再度 putbox() で置き直す
  place()
  
  別のデモ
  from sample_handcam import *
  putbox(name="box0", vaxis="y")
  putbox(name="box1", vaxis="y")
  handcam_demo()
  
  動作を実行し直すには,
  r.reset_pose()
  の後，putbox以降を実行する．

ビューアは，右ドラッグで回転，中ドラッグで拡大・縮小．

=============
RTMとして使う
=============
::

  ネームサーバを起動する
  $ rtm-naming
  動作生成RTCを起動する
  $ roscd iv_plan/src; ./MPlanComp.py -f ../conf/rtc.conf
  rtc-handleを利用して，pythonシェルからサービスを呼ぶ
  $ roscd iv_scenario/src; ipython sample.py
  目標手先フレームの生成(RTC.Pose3D)
  f = gen_goal_frm(y=-250)
  RTCサービス呼出し
  plsvc.ref.MoveArm(f,100,'right',False,False)



========
補足説明
========
rにはロボット，envには環境，plにはプランナがbindされています．
物体は名前で管理されています．
::

  オブジェクト一覧
  env.get_objects()
  名前で取得
  b=env.get_object('box0')
  オブジェクトのフレームを取得（返り値はFRAME）
  b.where()
  オブジェクトの削除
  env.delete_object('box0')
  
  あとは，
  r.
  env.
  などでTABを押すとどんなメソッドがあるかわかり，
  help r.grasp
  などで関数（メソッド）のインタフェースがわかります．

