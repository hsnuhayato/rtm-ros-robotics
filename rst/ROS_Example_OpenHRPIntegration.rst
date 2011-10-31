*summary OpenHRP integration*

==========
コンパイル
==========
::

  rosmake  --rosdep-install --rosdep-yes hrpsys_ros_bridge

--------------------------------
サンプルロボットシミュレーション
--------------------------------
::

  roslaunch hrpsys_ros_bridge samplerobot.launch
  roslaunch hrpsys_ros_bridge samplerobot_ros_bridge.launch


hrpsyspyクライアント
::

  rosrun hrpsys SampleRobot_walk.sh 


JointTrajectoryAction_ を使ってロボットの関節を動かせるようになっている．

.. _JointTrajectoryAction: http://www.ros.org/wiki/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action

euslisp クライアント
::

  rosrun roseus roseus `rospack find hrpsys_ros_bridge`/scripts/samplerobot-pickup.l


----------------------
HRP4C シミュレーション
----------------------

::

  rosmake --rosdep-install hrpsys

を行っている途中で、hrp4cのモデルをダウンロードするページが
表示されるので、そのページで登録を行ってダウンロードを行う必要がある。
~/Download　以下においてもう一度rosmakeを行うと
自動的に展開される。

RTM通信で表示されるopenhrp3とROSとが通信するためのパッケージ
hrpsys_ros_bridgeを作り、
センサ表示をROSメッセージを表示するrvizを用いて行う。

::

  roslaunch hrpsys_ros_bridge hrp4c.launch

として，シミュレーションが起動したら別のターミナルを立ち上げ
::

  roslaunch hrpsys_ros_bridge hrp4c_ros_bridge.launch


-----------------------
DARWIN シミュレーション
-----------------------

::

  roslaunch hrpsys darwin.launch
  # シミュレーション開始ボタンをおす
  # Execute script file ボタンをおす
  roslaunch hrpsys_ros_bridge darwin_ros_bridge.launch
  roscd hrpsys/scripts/Darwin
  roseus sample.l

::

  うまく行かない時は，
  roscd rtmros_common; svn up
  rosmake hrpsys_ros_bridge
  rosmake hrpsys
  roscd hrpsys/scripts/Darwin; make
  などを行っておく．

-----------------------------------------
RH (リファレンスハード） シミュレーション
-----------------------------------------

天井画像を見てナビゲーションを行う行動

簡易マニュアル_

.. _簡易マニュアル: 簡易マニュアル.pdf

移動ロボット( 機能仕様書_ )

.. _機能仕様書: doc20110629.zip

::

  roslaunch RS003 rh.launch
  # シミュレーション開始ボタンをおす
  # Execute script file ボタンをおす

::

  roscd rtmros_common; svn up
  rosmake RS003  
  を行っておく必要がある．

