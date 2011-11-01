RTミドルウェア
================

RTミドルウェアとは？
------------------------

- 以下の文書を抜粋したURL

  http://rtm-ros-robotics.googlecode.com/svn/wiki/20110420-rtmros-yoshikai.pdf

  分散オブジェクト指向システムのための技術であるCORBAをベースにしたRT(Robot Technology)分野のアプリケーションの共通ミドルウェアを提供する枠組み
  現在，本体に関しては産総研のグループを中心に開発が進められている。
  NEDO次世代ロボット知能化技術開発プロジェクト(H20-H23)において，RTミドルウェアをコアとした知能化
  ソフトウェアモジュール群の開発が全国の企業大学で行われている

 - オフィシャルページ http:// www.openrtm.org/

 - 現在の所，公式ページのドキュメント類は日/英/韓に対応

- 以下の文書を抜粋したURL

 http://www.openrtm.org/OpenRTM-aist/html/RTE3839FE38389E383ABE382A6E382A8E382A2E381A8E381AFEFBC9F.html

  インターネットの広がりとともに、ロボットやロボットシステムをネットワーク化しネットワーク上のリソース
  を活用して、更なる知能化を目指す研究・開発が盛んに行われています。しかしながら、これらのシステムの開
  発には通常膨大な開発技術者の投入及び、長い開発期間が必要で、ロボット製品として市場に提供できるだけの
  レベル、機能、価格のものはいまだほとんど世に出ていないのが現状です。
  RTミドルウエアは、様々なロボット要素（RTコンポーネント）を通信ネットワークを介して自由に組み合わせる
  ことで、多様なネットワークロボットシステムの構築を可能にする、ネットワーク分散コンポーネント化技術に
  よる共通プラットフォームを確立することを目指しています。 
  ここでいうロボットシステムとは、必ずしも移動ロボットやヒューマノイドロボットといった単体のロボットの
  みを想定している訳ではなく、「ロボット技術を活用した、実世界に働きかける機能を持つネットワーク化され
  た知能化システム」の総体としてのロボットシステムを指しています。例えばセンサ、アクチュエータを生活空
  間の中に分散配置させ、ネットワークを介して協調することにより生活支援や介護を実現するといった、一見ロ
  ボットには見えないがロボット的な技術を利用したシステムを広く包含しています。実際、センシング技術とセ
  ンサから得られる信号の処理、及びアクチュエータ等による現実世界への働きかけのフィードバックにより相互
  作用を行うシステムはロボット技術といってよいでしょう。日本ロボット工業会が主体となり、こうしたロボッ
  ト技術の総称を RT(RobotTechnology) と呼ぶことが提言されています。
  RTミドルウエアはこうした技術(RT)の共通プラットフォームを整備し、ロボット研究・開発の効率を高め、RTの
  適用範囲を広げ、さらには新たな市場を拓くことを目指して現在も開発が続けられています。ロボットシステム
  のソフトウェア開発においては、通常のソフトウェア開発と比べて、ロボットシステム特有の問題のためにソフ
  トウェアの再利用性が低く開発効率が悪いといった問題が指摘されています。こうした背景から、ロボット技術
  要素をソフトウェアレベルでモジュール化し、その再利用性を高めるミドルウェアを研究・開発し多くのユーザ
  に使ってもらい、さらには開発に参加してもらうことで、ロボットシステム開発に資する共通プラットフォーム
  を提供することがRTミドルウエアの目的なのです。


RTミドルウェアのプログラミングモデル
----------------------------------------

.. image:: images/RTCBuilder1.*


これはキャプションです。

**Ｔ．Ｂ．Ｄ**

RTミドルウェアを使ってみよう！
----------------------------------

インストール
~~~~~~~~~~~~~~~~~~

1 RTMミドルウェアのインストール

- C++版

 - Windows
  インストーラを利用する．Visual Studio2008ならば，
  http://www.openrtm.org/pub/Windows/OpenRTM-aist/cxx/OpenRTM-aist-1.0.0-RELEASE_vc9_100212.msi
  をダウンロードして，標準を選択して，インストールを実行する．
  その他のもの（Vineなど他のLinuxディストリビューションやMacOSなど）はhttp://www.openrtm.org/openrtm/ja/node/849#toc2 を参照のこと． 

 - Linux
  一括インストールスクリプトを利用する。

  :: 
  
    > wget http://www.openrtm.org/pub/OpenRTM-aist/cxx/install_scripts/pkg_install100_ubuntu.sh
    > sudo sh ./pkg_install100_ubuntu.sh

  途中，いくつかの質問をたずねられるので，''y'' あるいは ''Y'' を入力しながら完了させる． 

- Python版

 - Windows
  インストーラを利用する
  http://www.openrtm.org/pub/Windows/OpenRTM-aist/python/OpenRTM-aist-Python-1.0.1.msi
  をダウンロードして，標準を選択して，インストールを実行． 

 - Linux
  一括インストールスクリプトを利用する。

  ::
  
    > wget http://www.openrtm.org/pub/OpenRTM-aist/python/install_scripts/pkg_install_python_ubuntu.sh
    > sudo sh ./pkg_install_python_ubuntu.sh

- Java版

 - Windows
  **Ｔ．Ｂ．Ｄ**

  http://openrtm.org/openrtm/ja/node/933

 - Linux
  **Ｔ．Ｂ．Ｄ**

  http://openrtm.org/openrtm/ja/node/933

2 RTCBuilder/RTSystemEditorのインストール

 eclipseからグラフィカルにコンポーネントを生成するツールとしてRTCBuilder，操作するツールとして
 RTSystemEditorが公開されている． 

- Linux

 公式ページにおいてあるEclipse3.4.2の全部入りパッケージでは，そのままでは9.10で以降で動かない
 ため，Eclipse3.6にバイナリのjarファイルを展開して用いるのが良い．そのためのシェルスクリプトを
 用意したので，それをダウンロードして実行する． 

 ::

   > sh ./setup-eclipse.sh

 また，ipv6の設定によりlocalhostという名前解決ができないことがあるため，その場合，root権限で
 /etc/hostsの5行目をコメントアウトする．即ち， 

 ::
 
   ::1     localhost ip6-localhost ip6-loopback   

 を
 
 ::
 
   #::1     localhost ip6-localhost ip6-loopback   

 と変更して保存する． 

- RTCBuilderの説明

  http://openrtm.org/openrtm/ja/node/1176

- RTCSystemEditorの説明
  http://www.openrtm.org/OpenRTM-aist/html/E3839EE3838BE383A5E382A2E383AB2FRTSystemEditor.html

サンプルプログラム
~~~~~~~~~~~~~~~~~~~~~~~~

- Hello World

  **Ｔ．Ｂ．Ｄ**

  http://code.google.com/p/rtm-ros-robotics/wiki/RTM_HelloWorldSample

- Kinect

  **Ｔ．Ｂ．Ｄ**

- OpenCVでの顔認識

  **Ｔ．Ｂ．Ｄ**

  http://code.google.com/p/rtm-ros-robotics/wiki/RTM_Example

