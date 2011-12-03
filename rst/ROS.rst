ROS
===

`ROSとは？`
-----------

- 以下の文書を抜粋したURL

 http://rtm-ros-robotics.googlecode.com/svn/wiki/20110420-rtmros-okada.pdf

 - 2007年Switchyard Stanford AI研M.Quigley
 - 2008年WillowGarage社がサポート開始,Brian Gerkey（Stage/Player）がWG社に参画
 - ロボットアプリケーションを作成するソフトウェア開発者のためのライブラリとツール．ハードウェア抽象化，デバイスドライバ，ライブラリ，視覚化ツール，メッセージ通信，パッケージ管理等
 - http://www.ros.org/news/2010/11/happy-3rd-anniversary-ros.html
 - http://www.willowgarage.com/

- 以下の文書を抜粋したURL

 http://www.ros.org/wiki/ja/ROS/Introduction

  ROSはあなたのロボットのための，オープン・ソースのメタ-オペレーティング・システムです．ROSはあなた
  がオペレーティングシステムに望んでいたであろう，ハードウェア抽象化や低レベルデバイス制御・よく使
  われる組み込み関数・プロセス間通信・パッケージ管理の機能を持っています．さらにROSは多様な
  コンピュータ間を横断して適用したり，ビルドしたり，記述したり，実行したりするコードのためのツール
  やライブラリも提供します．「ROS」はPlayerやYARP・CARMEN・Orca・MOOS・Microsoft Robotics Studioと
  いった，”ロボットフレームワーク”といくつかの点で似ています．
  ROSのランタイム"graph"はROSコミュニケーション・インフラを用いて接合されたプロセス同士の，ゆるい
  Peer-to-Peerネットワークで成り立っています．ROSには，サービスごしの同期RPC形式の通信やトピックご
  しの非同期データ・ストリーミング・パラメータ サーバ上のデータ・ストレージといった，幾つかの異なる
  タイプの通信方法が実装されています．これらについてはROS のコンセプトに詳細な説明があります．
  ROSはリアルタイムのフレームワークではありませんが，ROSにはリアルタイムのコードを含めることができます．
  Willow GarageのPR2ロボットはpr2_etherCATと言うシステムを使用していますが，これはリアルタイム・プロ
  セスへROSのメッセージを送受信するものです．さらにROSはOrocosのリアルタイム・ツールキットとシームレス
  に統合します． 

  代表的なROSのスタックリストの紹介も含む

    http://www.ros.org/wiki/StackList

ROSのプログラミングモデル
-------------------------

*Ｔ．Ｂ．Ｄ*

`ROSを使ってみよう！`
---------------------

インストール
^^^^^^^^^^^^
1 ROS本体のインストール

 - Linux

  http://www.ros.org/wiki/electric/Installation/Ubuntu

  より抜粋

  ROS本体、Robot向け汎用パッケージ、PR2向けパッケージなど公開されているパッケージをソースでインス
  トールする。まずは以下の様にaptのリポジトリを登録し，

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

2 ROS外部パッケージのインストール

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


3 ROS関連コマンド

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

 等としてサンプルを試すことが出来る．crank-motionはリターンキーを打つと止まり、irteusgl$のプロンプトが
 出る．プロンプトに対して、exitを入れるとeuslispが終了する。

 より詳しい情報は http://www.ros.org/wiki/ROS/Tutorials へ，また，

 いざとなったら `ROS CheetSheet`_ を参考にすると助けになる．

.. _`ROS CheetSheet`: http://www.ros.org/wiki/Documentation?action=AttachFile&do=get&target=ROScheatsheet.pdf

 ほとんどのunixコマンドは先頭にrosをつけたツールが存在する(rosls, roscp等）．いろいろと調べるとよいが，
 もしかしたらその先は 奥深い_ ので要注意．

.. _奥深い: http://0xcc.net/misc/bad-knowhow.html

 インストールができたら， サンプルプログラム_ を試してみよう

.. _サンプルプログラム: ROS_Example.html



サンプルプログラム
~~~~~~~~~~~~~~~~~~
- Hello World

  **Ｔ．Ｂ．Ｄ**

  http://code.google.com/p/rtm-ros-robotics/wiki/ROS_Example

