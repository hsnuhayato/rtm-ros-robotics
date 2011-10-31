*summary ソフトウェアのテスト・ドキュメント・リリースの練習*

この宿題の目的は丈夫なソフトウェアをリリースするためのテスト、ドキュメント、リリース過程を通ることです。

====
課題
====

- （１点）画像から円の位置を復元出来るROSノードの作成

- （５点）楕円抽出の単体テスト、ROSノードの結合テスト

- （３点）ドキュメントの作成

- （１点）Debianパッケージの作成

合計点数：１０点
締切り：5月23日23:59時

==================
プロジェクトの仕様
==================

世界に緑色の円が決まった大きさで存在しています。その円がイメージに投影されると楕円として表示されます。画像から緑色の楕円を検出し、もとの３次元の円の位置を計算し、その情報をROSに送信するのがプロジェクトの目的です。

-------------
１。ROSノード
-------------

~~~
API
~~~

fitEllipseグローバル関数：楕円を近似する
::

  fitEllipse(points) -> ellipse

fitEllipseFromImageグローバル関数：楕円を画像から抽出する
::

  fitEllipseFromImage(image) -> ellipses

DetectEllipseNodeクラス：ROSノードを立ち上げ、メッセージとサービスをやりとりする。

~~~~~~~~~~~~
パラメーター
~~~~~~~~~~~~

ellipse_radius - 円の物理的な大きさ

パラメーター仕様のチュートリアル_

.. _パラメーター仕様のチュートリアル: http://www.ros.org/wiki/roscpp/Overview/Parameter%20Server

~~~~~~~~~~~~~~
入力メッセージ
~~~~~~~~~~~~~~

image - `sensor_msgs/Image`_

.. _`sensor_msgs/Image`: http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html

~~~~~~~~~~~~~~
出力メッセージ
~~~~~~~~~~~~~~

画像からの全楕円の2D位置と回転。 このメッセージをパッケージに定義する_ 
メッセージ名前：Poses2D

.. _このメッセージをパッケージに定義する: http://www.ros.org/wiki/ROS/Tutorials/CreatingMsgAndSrv

::

  geometry_msgs/Pose2D[] poses

緑色の楕円の３次元中心点：
メッセージ名前： `sensor_msgs/PointCloud`_

.. _`sensor_msgs/PointCloud`: http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud.html

~~~~~~~~
サービス
~~~~~~~~

画像を入力し、３次元点を返すサービス：

名前：DetectCircles
::

  sensor_msgs/Image image
  ---
  sensor_msgs/PointCloud circles

サービス使用チュートリアル_

.. _サービス使用チュートリアル: http://www.ros.org/wiki/roscpp_tutorials/Tutorials/WritingServiceClient

サービス作成情報_

.. _サービス作成情報: http://www.ros.org/wiki/ROS/Tutorials/CreatingMsgAndSrv

----------
２。テスト
----------

~~~~~~~~~~
単体テスト
~~~~~~~~~~

- 2次元の点群から楕円の２D位置と大きさの計算

- 画像から楕円の２D位置と大きさが検出されているか

- 円の３次元位置が計算出来るか

テストデータに二つの種類があります。

1. ランダムに点群と画像を作成する
2. 撮影された画像をファイルから読み込む。楕円の位置を他のファイルに保存する必要があります。

関数：

- fitEllipse

- fitElliseFromImage

- DetectEllipseNode::DetectEllipses

- DetectEllipseNode::DetectCircles

~~~~~~~~~~
結合テスト
~~~~~~~~~~

テストする時にROSコアと画像を供給するプログラムも立ち上げます。

- ROSメッセージで画像を供給し、結果が出力されているか

- ROSのDetectCirclesサービスが動いているか


~~~~~~~
Jenkins
~~~~~~~

- Jenkins_ を自分のPC上に立ち上げる。

- テストが毎日の夜に走るように設定を行う。

- テスト結果が見えるように設定を行う。

.. _Jenkins: http://jenkins-ci.org/

----------------
３。ドキュメント
----------------

- 紹介ページを作る。

 - 目的

 - 使い方

 - 実行例：画像も添付する。（rvizの表示画像）

- 関数とクラスの必要なところがコメントされる。

 - *注意* どこにコメントが必要かを考える必要があります。不要なところにコメントが書かれると、読者の時間の無駄になるので点数が引かれます。

- テスト結果の表示（Jenkins）

 - プログラムがすごくよく動いていることを納得させるためです。

 - テストをみたらプログラムの限界が分かるようにしてください


--------------------
４。Debianパッケージ
--------------------

バイナリーDebianパッケージをCMakeとCPackから作ることです。

==========
準備・設定
==========

ROS_Install_ でagentsystem_ros_tutorialsのチェックアウトしているはずです。以下のように更新し、agentsystem_ros_tutorials/opencv_fittingを参照し、個人のレポジトリーにコピーください。

.. _ROS_Install: ROS_Install.html

::

  roscd agentsystem_ros_tutorials
  svn up
  svn export opencv_fitting ~/prog/myrepo/mystack/opencv_fitting
  svn add ~/prog/myrepo/mystack/opencv_fitting

opencv_fittingは

- APIをinclude/fitting.hから提供している、

- libfittingのライブラリを作成している、

- fitting_nodeのプログラムを作成している、

- test/testfitting.cppのテストを登録している。

----------
楕円の検出
----------

OpenCVの fitellipseのサンプル_ は画像から楕円を抽出しているのでそれをROS化するのが一番速いです。

.. _fitellipseのサンプル: https://code.ros.org/trac/opencv/browser/trunk/opencv/samples/cpp/fitellipse.cpp

円の３次元位置を正確に計算するのにカメラの キャリブレーションパラメター_ が必要です。それが手間がかかるのでカメラの焦点距離を推測しても結構です。

.. _キャリブレーションパラメター: http://www.ros.org/wiki/image_pipeline/CameraInfo

------------
テストの設定
------------

gtest_ と rostest_ を参照してくだい。test_roscppのテストもとても勉強になります。

.. _gtest: http://www.ros.org/wiki/gtest

.. _rostest: http://www.ros.org/wiki/rostest

::

  roscd test_roscpp


test/CMakeLists.txtで

- rosbuild_add_gtestを単体テストに使用

- rosbuild_add_rostestを結合テストに使用


以下のコマンドで実行される：
::

  make test
  rostest ...

テストの結果はopencv_fitting/test/opencv_fittingに保存されています。

Jenkinsをローカルとにインストールしてください。
Jenkinsインストールチュートリアル_

.. _Jenkinsインストールチュートリアル: https://wiki.jenkins-ci.org/display/JENKINS/Installing+Jenkins+on+Ubuntu

設定内容

- 新しいノードを作ってSSHで自分のユーザーにログインする

- 一つのジョブを登録

 - ジョブで個人レポジトリーをチェックアウトし

 - ROS_PACKAGE_PATHの変更

 - "rosmake opencv_fitting"を呼び出し

 - テストを呼び出す

 - 結果を収集し、Jenkinsの画面に表示されるように設定する。

  - JUnitテスト結果の集計


テスト画像のサンプルが hw3_testimages.tgz_ で入手出来ますが、 *自分のテストセットを必ず作る必要があります* 。

.. _hw3_testimages.tgz: hw3_testimages.tgz

.. image :: hw3_testimage2.jpg

------------
ドキュメント
------------

作成法は rosdoc_ を参照してください。一番簡単な方法は

.. _rosdoc: http://www.ros.org/wiki/rosdoc

::

  rosrun rosdoc rosdoc opencv_fitting

依存しているパッケージもドキュメントに参照出来るので試してみてください。例えばOpenCVの関数を参照してみてください。

C++で重要なファイル

- include/fitting.h

- mainpage.dox

Pythonを使う予定だったらsrc/opencv_fitting/fitting.pyになります。

----------------
Debianパッケージ
----------------

makeが終わる時に

::

  cd build
  cpack  -G DEB .

を実行すると

Project-0.1-Linux.debが出来上がります。

========
提出項目
========

全ファイルを個人レポジトリーの
::

  mystack/opencv_fitting

においてください。

コンパイルとテストが通っているかが調べられます。

::

  rosmake opencv_fitting
  rosmake opencv_fitting --test-only

HTMLドキュメントをrosdocで調べられます。テスト結果が出来ているか、システムを説明する画像が出ているかが調べられる。

Debianパッケージの作成もcmake/cpackで調べられます。宿題提出のために*.debファイルをコードにコミットしてください。

----
言語
----

C++かPythonを使ってもOKです。両方で完璧に出来ている人は５点で増えるので挑戦してみてください。
