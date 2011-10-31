*summary ROSインストール法*

.. <wiki:toc max_depth="2" />

=====================
ROS本体のインストール
=====================

.. <wiki:comment>
   http://www.ros.org/wiki/diamondback/Installation/Ubuntu
   </wiki:comment>

http://www.ros.org/wiki/electric/Installation/Ubuntu
 より抜粋


ROS本体、Robot向け汎用パッケージ、PR2向けパッケージなど公開されているパッケージをソースでインストールする

まずは以下の様にaptのリポジトリを登録し，

::

  sudo apt-get install build-essential python-yaml cmake subversion wget python-setuptools git-core mercurial aptitude
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

次に以下の作業を行う．

::

  sudo apt-get update
  sudo apt-get install python-setuptools
  sudo easy_install -U rosinstall
  sudo apt-get install ros-electric-wg-pr2-apps ros-electric-pr2-apps ros-electric-pr2-desktop ros-electric-openni-kinect

.. <wiki:comment>
   sudo apt-get install ros-diamondback-wg-pr2 ros-diamondback-pr2-desktop ros-diamondback-joystick-drivers ros-diamondback-sound-drivers ros-diamondback-openni-kinect
   </wiki:comment>

11.04ユーザは，

::

  sudo apt-get install xfonts-100dpi xfonts-75dpi msttcorefonts

をしてリブートしておく必要がある．


===============================
ROS外部パッケージのインストール
===============================


::

  rosinstall ~/prog/rtm-ros-robotics /opt/ros/electric http://rtm-ros-robotics.googlecode.com/svn/tags/latest/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall


.. <wiki:comment>
   rosinstall ~/prog/rtm-ros-robotics /opt/ros/diamondback http://rtm-ros-robotics.googlecode.com/svn/tags/latest/agentsystem_ros_tutorials/rtm-ros-robotics.rosinstall
   </wiki:comment>

rosinstallについては http://www.ros.org/wiki/rosinstall を参照

また，

::

  rosinstall ~/prog/rtm-ros-robotics 

とすると，ソースツリーを更新してくれる．

ROS関連のプログラムを使うためには以下の様にして環境変数をセットする必要があります．

::

  source ~/prog/rtm-ros-robotics/setup.bash

この一行を ~/.bashrc に追加するとよいでしょう．

===============
ROS関連コマンド
===============

最低限 roscd_ , rosdep_ , rosmake_ , rosrun_ を知っていればよい．例えば，

.. _roscd: http://www.ros.org/wiki/roscd

.. _rosdep: http://www.ros.org/wiki/rosdep

.. _rosmake: http://www.ros.org/wiki/rosmake

.. _rosrun: http://www.ros.org/wiki/rosrun

::

  rosdep install euslisp
  rosmake euslisp

とすると， euslisp_ パッケージに必要なパッケージをダウンロード＆インストールし，さらにコンパイルする．

.. _euslisp: http://jskeus.sourceforge.net/

また，euslispパッケージにあるディレクトリに移動したければ

::

  roscd euslisp

とすればよい．また，euslispパッケージ以下のirteusglという実行ファイルを実行したければ

::

  rosrun euslisp irteusgl

とする．引数も渡せるので，

::

  rosrun euslisp irteusgl irteus/demo/demo.l "(crank-motion)"
  rosrun euslisp irteusgl models/irt-all-robots.l "(make-all-robots)"
  rosrun euslisp irteusgl models/irt-all-objects.l "(make-all-objects)"

等としてサンプルを試すことが出来る．crank-motionはリターンキーを打つと止まり、irteusgl$のプロンプトが出る．プロンプトに対して、exitを入れると
euslispが終了する。

より詳しい情報は http://www.ros.org/wiki/ROS/Tutorials へ，また，

いざとなったら `ROS CheetSheet`_ を参考にすると助けになる．

.. _`ROS CheetSheet`: http://www.ros.org/wiki/Documentation?action=AttachFile&do=get&target=ROScheatsheet.pdf

ほとんどのunixコマンドは先頭にrosをつけたツールが存在する(rosls, roscp等）．いろいろと調べるとよいが，もしかしたらその先は 奥深い_ ので要注意．

.. _奥深い: http://0xcc.net/misc/bad-knowhow.html

インストールができたら， サンプルプログラム_ を試してみよう

.. _サンプルプログラム: ROS_Example.html

