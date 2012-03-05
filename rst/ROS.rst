ROS
===

`ROSとは？`
-----------

ROSはロボット向けのオペレーティングシステムである．一般のオペレーティン
グシステムはハードウェアの抽象化，デバイス制御，標準機能の提供，プロセ
ス間通信，パッケージ管理等のサービスをユーザに提供するが，ROSはこれらの
機能をロボット研究者・開発者向けに提供するものである．

ロボット用ミドルウェアという観点からみるとROSの特徴は出版・購読型の分
散オブジェクト通信機構にある．これは送信者は特定の購読者を
指定することなくメッセージを送出し，購読者が興味ある内容
や経路を登録し，これに合致したものだけを受け取る機構になっている．

http://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern

ロボット研究者・開発者向けという観点から見るとROSの特徴はソースコード
やライブラリの再利用性の向上に関わる工夫が随所にちりばめられているとい
う点に見て取れる．後述するソースコードをグ
ルーピングするためのパッケージやスタックを扱うツールや，これらのソース
コード群を公開されているリポジトリを自身のソフトウェア開発に活用
するためのツール等がその代表になり，また開発者コミュニティをサポートす
るための様々なツールや場の提供もROSの重要な役割の一つである．

このように，ROSは多様的な側面を持つために全体像を把握するのが困難であ
るが，中心開発者の一人であるBrian Gerkeyは以下の様に
通信機構(plumbing) + ツール(tools) + ライブラリ群(capabilities) +
コミュニティ(ecosystem)から構成されるプロジェクトであると定義している．

http://answers.ros.org/question/3195/what-is-ros-exactly-middleware-framework-operating

 ::

  I usually explain ROS in the following way:
  
  ROS = plumbing + tools + capabilities + ecosystem
  
   1.
        plumbing: ROS provides publish-subscribe messaging
        infrastructure designed to support the quick and easy
        construction of distributed computing systems.
   2.
      tools: ROS provides an extensive set of tools for configuring,
      starting, introspecting, debugging, visualizing, logging,
      testing, and stopping distributed computing systems.
   3.
      capabilities: ROS provides a broad collection of libraries that
      implement useful robot functionality, with a focus on mobility,
      manipulation, and perception.
   4.
      ecosystem: ROS is supported and improved by a large community,
      with a strong focus on integration and documentation. ros.org is
      a one-stop-shop for finding and learning about the thousands of
      ROS packages that are available from developers around the
      world.
  
  In the early days, the plumbing, tools, and capabilities were tightly
  coupled, which has both advantages and disadvantages. On the one hand,
  by making strong assumptions about how a particular component will be
  used, developers are able to quickly and easily build and test complex
  integrated systems. On the other hand, users are given an "all or
  nothing" choice: to use an interesting ROS component, you pretty much
  had to jump in to using all of ROS.

  Four years in, the core system has matured considerably, and we're
  hard at work refactoring code to separate plumbing from tools from
  capabilities, so that each may be used in isolation. In particular,
  we're aiming for important libraries that were developed within ROS to
  become available to non-ROS users in a minimal-dependency fashion (as
  has already happened with OMPL and PCL).
  
  posted Dec 06
  Brian Gerkey


- 現在利用されているロボット
- どういうライブラリが使われているか．
- どういうパッケージがあるか

より詳細な説明はhttp://www.ros.org/wiki/ROS/Introductionを参照するとよ
い．


ROSの歴史
^^^^^^^^^

 - 2007年Switchyard Stanford AI研M.Quigley
 - 2008年WillowGarage社がサポート開始,Brian Gerkey（Stage/Player）がWG社に参画
 - http://www.ros.org/news/2010/11/happy-3rd-anniversary-ros.html
 - http://www.willowgarage.com/


ROSの用語集
^^^^^^^^^^^^^^^

http://www.ros.org/wiki/ROS/Concepts 

- ファイルシステム

 The filesystem level concepts are ROS resources that you encounter on
 disk, such as

 - パッケージ(Package)
 - マニュフェスト(Manifests)
 - スタック(Stacks)
 - スタックマニュフェスト(Stack Manifests)
 - メッセージタイプ(Message types)
 - サービスタイプ(Service types)

- 分散オブジェクト通信レベル
 The Computation Graph is the peer-to-peer network of ROS processes
 that are processing data together. The basic Computation Graph
 concepts of ROS are nodes, Master, Parameter Server, messages,
 services, topics, and bags, all of which provide data to the Graph in
 different ways.

 - ノード(Nodes)
 - マスター(Master)
 - パラメータサーバ(Parameter Server)
 - メッセージ(Messages)
 - トピック(Topics)
 - サービス(Services)
 - バグ(Bag)

- コミュニティレベル
 The ROS Community Level concepts are ROS resources that enable
 separate communities to exchange software and knowledge. These
 resources include: 

 - ディストリビューション(Destributions)
 - リポジトリ(Repositories)
 - ウィキ(The ROS Wiki)
 - 質問サイト(ROS Answers)


`ROSを使ってみよう！`
---------------------

インストール
^^^^^^^^^^^^

 - Linux

  http://www.ros.org/wiki/electric/Installation/Ubuntu

  より抜粋

  まずは簡単なサンプルプログラムを実効するために必要なパッケージをインス
  トールする。まずは以下の様にaptのリポジトリを登録し，rosinstallをイ
  ンストールする．

  .. shell-block:: bash -c '$ROS_WORKSPACE/jsk-ros-pkg/jsk.rosbuild -i'

  次に以下の作業を行い必要なROSファイルをインストールする

  ::

    sudo apt-get install ros-electric-ros-tutorials ros-electric-rx

  これができたら，
  ::

    echo "source /opt/ros/electric/setup.bash" >> ~/.bashrc
  として

  ::

    $ source ~/.bashrc


  としてROS環境のセットアップを行う．これにより以下の重要な環境変数
  (http://www.ros.org/wiki/ROS/EnvironmentVariables)
  が設定され，基本的なROSコマンド
  (http://www.ros.org/wiki/ROS/CommandLineTools)が使えるようになる．

  rosversionというコマンドを打ち込み以下の様な結果が得られたらインストー
  ルと環境設定は成功である．

  ::

    $ rosversion  -d
    electric


ROS_PACKAGE_PATH : ROSのパッケージを検索するパスを設定する

ROS_MASTER_URI   : ROSのマスターを指定する(デフォルトはhttp://localhost:11311/
                   である．localhostを他人のPCのIPアドレスにすると，その人のPC
                   のマスターを利用する）

ROS_IP/ROS_HOSTNAME : ROSのノードのネットワークアドレスを指定する．
                      外部のPCと接続する場合はそのPCから自分のPCを一意に指定
                      できるようにホスト名，あるいはIPを指定しておく．

簡単なサンプルプログラム
^^^^^^^^^^^^^^^^^^^^^^^^

  簡単な通信プログラムの例として
  Python(http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber(python))
  と
  C++(http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber(c++))
  を例に見てみよう．

  ここではrospy_tutorialsというパッケージ以下の001_talker_listener/talker.py
  というファイル名のPythonプログラムがメッセージを送信する出版
  側となり，roscpp_tutorials/listener/listener.cppというファイル名の
  C++ファイルをコンパイルして生成されるroscpp_tutorials/bin/listenerと
  いうプログラムがメッセージを受信する購読側となる．


  それぞれ以下の様にしてファイルを確認することができる．

  ::

    $ rosls rospy_tutorials/001_talker_listener
    README  listener  listener.py  listener.pyc  talker  talker.py
    talker.pyc  talker_listener.launch


  ::

    $ rosls roscpp_tutorials/listener
    CMakeLists.txt  listener.cpp

  出版側のPythonプログラムの起動は


  ::
    $ roscd rospy_tutorials/001_talker_listener
    $ python ./talker.py

  として通常のPythonプログラムのように起動することも可能だし，rosrunコ
  マンドを利用して


  ::

    $ rosrun rospy_tutorials talker.py

  として起動してもよい．

  ::

    $  rosrun rospy_tutorials talker.py      
    [ERROR] [WallTime: 1324881554.367677] Unable to immediately register
    with master node [http://localhost:11311]: master may not be running
    yet. Will keep trying.

  ここで，上記のメッセージが表示される場合はroscoreが起動していない．
  もう一つターミナルを立ち上げて

  ::

   $ roscore

  としてroscoreを立ち上げると，元のtalker.pyを立ち上げてターミナルで以
  下のようなメッセージが流れ出すはずである．


  ::

    [INFO] [WallTime: 1324881648.660502] hello world 1324881648.66
    [INFO] [WallTime: 1324881648.760936] hello world 1324881648.76
    [INFO] [WallTime: 1324881648.860930] hello world 1324881648.86
    [INFO] [WallTime: 1324881648.960909] hello world 1324881648.96



  出版されたメッセージを購読するためにはrostopic コマンドを利用する．もう一つ
  新しいターミナルを立ち上げて

  ::

    $ rostopic echo /chatter
    data: hello world 1324882100.9
    ---
    data: hello world 1324882101.0
    ---
    data: hello world 1324882101.1
    ---
    data: hello world 1324882101.2


  としてデータを受信できる．これでchatterというトピックを介して
  talker.pyがメッセージを出版し，rostopicがこのメッセージを購読したこ
  とになる．

　なお，rostopicというコマンドを起動しようとして
  ::

    rostopic: command not found

  というエラー表示になった場合は~/.bashrcでsetup.bashをsourceしていな
  い場合が考えられる(前の記述とクロスリファレンス)．

  /chatterはトピック名と呼ばれる通信路につけられた名前である．現在のシス
  テムでどのようなトピックが提供されているかを確認するためには
  `rostopic info`コマンドを利用する

  ::

    $ rostopic  list
    /chatter
    /rosout
    /rosout_agg

  ここでは`chatter`, `rosout`, `rosout_agg`というトピックが出版されて
  いることが分かる．`rosout`, `rosout_agg`はデバッグ用のメッセージを流
  すためのトピック名である．

  また`rosnode list`コマンドを利用することで現在のシステムでどのような
  ROSノードが稼働しているかを確認することができる．

  ::

    $ rosnode  list
    /rosout
    /talker_22072_1324881936976
    /rostopic_23668_1324882765517

  /rosoutはシステムが自動で立ち上げるデバッグ用のノードであり，先ほど
  立ち上げたtalk.pyプログラムは/talker_22072_1324881936976  という名前
  で, rostopic プログラムは/rostopic_23668_1324882765517 という名前で
  登録されている(なぜ?を説明)．


  さらに，`rostopic info /chatter`とすると，このトピックで流れるメッセー
  ジ(型)の情報や，あるいはこのトピックの出版社や購読者に関する情報を得
  ることができる．

  ::

     $ rostopic info /chatter
     Type: std_msgs/String
     
     Publishers: 
      * /talker_22072_1324881936976 (http://localhost:35820/)
     
     Subscribers: 
      * /rostopic_23668_1324882765517 (http://localhost:51312/)



  例えば上記の表示は/chatterというトピックはstd_msgs/Stringというメッ
  セージ型で， localhost と   いう計算機上の
  talker_22072_1324881936976というROSノードから出版され，おなじ計算機
  上のrostpic_23668_1324882765517 というROSノードで購読されている．という
  ことを示している．

  `rosmsg show`コマンドを用いるとメッセージの詳細を知ることができる．
  std_msgs/Stringはdataという文字列を要素としてもつメッセージであることが分
  かる．

  ::

    $ rosmsg show std_msgs/String
    string data

  また

  ::

    $ rxgraph

   というコマンドで，現在のROSノードとトピックの関係を以下の様に視覚的に表示できる

   .. figure:: images/rxgraph_talker_listener.png


自分のパッケージを作成する
^^^^^^^^^^^^^^^^^^^^^^^^^^

  この節ではより実践的な例として，作業ディレクトリ
　として自分のパッケージを作成
  (http://www.ros.org/wiki/ROS/Tutorials/CreatingPackage)し，そこでサ
  ンプルプログラムを開発していこう．ここでは
  `~/ros_workspace`を作業ディレクトリとする．

  ::

    $ mkdir ~/ros_workspace


  作業ディレクトリをしたらこれをROS_PACKAGE_PATH環境変数にに追加する．
  ターミナルで

  ::

    $ export ROS_PACKAGE_PATH=$HOME/ros_workspace:$ROS_PACKAGE_PATH

  とするか，あるいはこれを~/.bashrcに書いて，source ~/.bashrcとする．次に，

  ::

    $ cd ~/ros_workspace
    $ roscreate-pkg beginner_tutorials std_msgs rospy roscpp


  としてROSのパッケージを作成する．これでいくつかのファイルが自動で作成され，

  ::

    $ roscd beginner_tutorials

  とすると，このディレクトリに移動できるようになる．

  出版側のプログラムは先ほどのPythonのサンプルコード(talker.py)を利用
  する．プログラムは次に示したようになっている．

  .. ros-file:: rospy_tutorials talker.py

  同様のプログラムを書いてbeginner_tutorialsディレクトリ以下に
  talker.pyファイルとして置き，

  ::

    chmod u+x ./talker.py

  として実行権限を出しておく．

  次に，C++でかかれた購読側のサンプルプログラムが次の
  プログラムになる．このプログラムをsrc/listener.cppに置く．

  .. ros-file:: roscpp_tutorials listener.cpp

  この時点でbeginner_tutorialsパッケージ以下は以下の様になっているはず
  である

  ::

     $ roscd beginner_tutorials 
     $ ls
     CMakeLists.txt  bin      mainpage.dox  msg  talker.py
     Makefile        include  manifest.xml  src
     $ ls src/
     listener.cpp

  これができたらCMakeLists.txtの最後に

  ::

    rosbuild_add_executable(listener src/listener.cpp)

  という記述を追加する．これにより，

  ::

     $ roscd beginner_tutorials
     $ make

  とするとsrc/listener.cppファイルがコンパイルされbin/listenerという実
  行ファイルが生成される．

  ::

    $ roscd beginner_tutorials
    $ ./bin/listener

  あるいは，
  ::

    $ rosrun beginner_tutorials listener

  とするとクライアントを実行できる．


メッセージの定義と利用
^^^^^^^^^^^^^^^^^^^^^^

  独自の型を追加したければmsgディレクトリ以下に

  ::

    $ cat msg/Hello.msg
    Header header
    string hello
    geometry_msgs/Vector3 pos

  として追加する．

  CMakeLists.txtの以下の行をコメントアウトしてmakeするとこのメッセージに
  対応したC++, Python等のコードが自動生成される．

  ::

    rosbuild_genmsg()

   talker.py, listener.cppを以下の様に改造すれば，独自に定義したメッセー
   ジをつかった通信が可能になる．

  ::
     
     #!/usr/bin/env python
     import roslib; roslib.load_manifest('beginner_tutorials')
     import rospy
     from std_msgs.msg import String
     from beginner_tutorials.msg import Hello
     
     def talker():
         pub = rospy.Publisher('chatter2', Hello)
         rospy.init_node('talker')
         while not rospy.is_shutdown():
             str = "hello world %s"%rospy.get_time()
             rospy.loginfo(str)
             hello = Hello();
             hello.hello = "world"
             hello.pos.x = 0;
             hello.pos.y = 1;
             hello.pos.z = 2;
             pub.publish(hello);
             rospy.sleep(1.0)
     if __name__ == '__main__':
         try:
             talker()
         except rospy.ROSInterruptException: pass

  ::
     
     #include "ros/ros.h"
     #include "std_msgs/String.h"
     #include "beginner_tutorials/Hello.h"
     
     void chatterCallback(const beginner_tutorials::Hello::ConstPtr& msg)
     {
       ROS_INFO("I heard: [%s %f %f %f]", msg->hello.c_str(),
                msg->pos.x, msg->pos.y, msg->pos.z);
     }
     
     int main(int argc, char **argv)
     {
       ros::init(argc, argv, "listener");
       ros::NodeHandle n;
       ros::Subscriber sub = n.subscribe("chatter2", 1000, chatterCallback);
       ros::spin();
       return 0;
     }



ナビゲーションサンプルプログラム
--------------------------------

  .. include::  ROS_Example_Navigation.rst

画像処理サンプルプログラム
--------------------------

  .. include:: ROS_Example_ImageProcessing.rst

RTMROS統合環境
--------------


.. include:: ROS_Example_RTMIntegration.rst

サンプルプログラム
~~~~~~~~~~~~~~~~~~

- Hello World

  **Ｔ．Ｂ．Ｄ**

  http://code.google.com/p/rtm-ros-robotics/wiki/ROS_Example


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


ROSは豊富な `Tutorialのページ <http://www.ros.org/wiki/ROS/Tutorials>`_, C++, Python以外にも, `CommonLisp <http://www.ros.org/wiki/roslisp>`_, `Java <http://www.ros.org/wiki/rosjava>`_, `Lua <http://www.ros.org/wiki/roslua>`_, `EusLisp <http://ros.org/doc/api/roseus/_tutorials/html/>`_, 等のクライアントライブラリがあり， `ロボット <http://www.ros.org/wiki/Robots>`_, 以外にも `Android <http://www.ros.org/wiki/ApplicationsPlatform/Clients/Android>`_, `Aruduino <http://www.ros.org/wiki/rosserial/_arduino>`_  などのデバイスが利用できる


