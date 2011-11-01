==========
4 OpenRave
==========

------------------
4.1 OpenRaveとは？
------------------

- 以下の文書を記述

 http://openrave.org/en/main/overview.html


----------------------------------
4.2 OpenRaveのプログラミングモデル
----------------------------------

- 以下の文書を記述

 http://openrave.programmingvision.com/en/main/install.html

----------------------------
4.3 OpenRaveを使ってみよう！
----------------------------

~~~~~~~~~~~~~~~~~~
4.3.1 インストール
~~~~~~~~~~~~~~~~~~

1 OpenRaveのインストール

- Linux

 Official Release PPA. Execute the following to add the OpenRAVE repository:

 ::

   sudo add-apt-repository ppa:openrave/release
   sudo apt-get update
   sudo apt-get install openrave

- Windows

 `Windows Installers`_ are compiled for every Visual Studio version.

.. _`Windows Installers`: http://sourceforge.net/projects/openrave/files/latest_stable

~~~~~~~~~~~~~~~~~~~~~~~~
4.3.2 サンプルプログラム
~~~~~~~~~~~~~~~~~~~~~~~~
- Hello World

 ** Ｔ．Ｂ．Ｄ **

 http://code.google.com/p/rtm-ros-robotics/wiki/OpenRAVE_Example

- マニピュレーション

 ** Ｔ．Ｂ．Ｄ **

 http://code.google.com/p/rtm-ros-robotics/wiki/OpenRAVE_Example

==================
5 ロボット開発実践
==================

---------------------------------------
5.1 Beegoによる移動タスクチュートリアル
---------------------------------------

====
概要
====
ここで紹介するナビゲーションのサンプルは，
http://www.ros.org/wiki/move_base_stage/Tutorials/stage%20and%20navigation%20stack 
に基づきました．ただ，このwikiのページは情報が古いので参考程度に見て下さい．紹介するナビゲーションサンプルは上のリンクのwikiページを見なくとも， ROS_Install_ をみて，インストールし，setup.bashをsourceしていれば，あとはこのページを読み進めていけば実行できます．


====
準備
====
ROS_Install_ にしたがってROSと講義関連パッケージがインストールされていることが前提です．

.. _ROS_Install: ROS_Install.html

====
実行
====

３つのターミナルを立ち上げます
まずは，1つ目で
::

  roscore

として下さい．

次に2つ目のターミナルで
::

  roslaunch move_base_stage_tutorial robot.launch

とします．
以下の様な初期画面が現れると思います．

.. image :: move_base_stage_tutorial_init.png

最後に
::

  rosrun rviz rviz -d `rospack find move_base_stage_tutorial`/config/rviz.vcg 

としてビューワを表示します．(エラーになる人はrosmake rvizを実行する)

ここで左上の2D Nav Goalを押して地図上の任意の場所をクリックすると
その場所まで移動します．

.. image :: move_base_stage_tutorial_rviz.png

また，
::

  rosrun pr2_teleop teleop_pr2_keyboard 

とすると，速度指令を送ることが出来ます．

==============
プログラミング
==============

速度指令プログラムは
::

  roscd pr2_teleop
  cat src/teleop_pr2_keyboard.cpp

をみるとよい．

----------------------------------------------------
5.2 HIROによるマニピュレーションタスクチュートリアル
----------------------------------------------------

 http://code.google.com/p/rtm-ros-robotics/wiki/ROS_English#Kawada_HIRONX
