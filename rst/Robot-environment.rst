ドキュメンテーションフレームワーク
==================================

ドキュメントの必要性
--------------------

 - ドキュメントを通してソフトウェア利用者と効率良くコミュニケーションをとることができる。
 - ドキュメントを作る行為は設計をすること
 - ドキュメントはメンテナンス時に必要なもの。

.. todo:: ドキュメントの必要性の記述を行う

reStructuredText / Sphinx
--------------------------

概要
~~~~

 reStructuredTextとは、文書の構造や表示の仕方などを定義したマークアップ言語のひとつである。
 reStructuredText は RST、ReST、reSTと略されることもある。
 reStructuredTextの特徴は以下の通り。

 - テキスト形式で記述するため、OSに標準でインストールされているメモ帳などのテキストエディタで作成することができる。
 - ソースコードの状態でも高い可読性を持っている。
 - reStructuredTextで記述された文書はPDF、HTML、XML、LaTeXなど複数の形式の文書に変換することができる。
 - reStructuredTextを他の形式の文書に変換するパーサは、テキスト処理フレームワークである Docutils のコンポーネントの一つであり、Pythonにより実装されている。 

.. figure:: images/RTCBuilder1.*

  reStructuredTextの記述例

.. todo:: reStructuredTextの記述例を表示する

 Sphinxはドキュメントを簡単に作れるようにするためのツールである。
 SphinxはreStructuredTextをマークアップ言語として使用しており、作成したドキュメントをHTML、LaTeX、PDFなどの形式に変換することができる。
 SphinxはreStructuredTextの特徴は以下の通り。

 - 実行環境はWindows/Linux/Mac OSであり、Python 2.4のインストールが必要である。
 - BSDライセンスを適用している。
 - reStructuredTextの変換プログラムはDocutilsを利用している。
 - 出力フォーマットはHTML、LaTeX、PDF、ePubなどをサポートしている。
 - 引用、用語解説、ソースコードへのクロスリファレンス機能。
 - 引用したソースコードを自動でハイライトする機能。
 - 章などのインデックスを自動で設定する機能。

導入
~~~~

 SphinxはPythonを利用して作成されていることから、SphinxのインストールにはPython、Sphinxの両方が必要となる。インストール手順(Windows編、Linux編)を以降に示す。

 1. Sphinxのインストール(Windows編)
  WindowsにSphinxをインストールする手順は以下の通り。
   - Pythonのインストール
   - easy_installのインストール
   - Sphinxのインストール

 1.1 Pythonのインストール
  Pythonのインストール手順を示す。なお、既にインストールされている場合は、本手順は不要である。

  (1) Pythonのインストーラのダウンロード
   - Internet Explorer等のWebブラウザを利用し、http://python.org の画面を開く。
   - 画面左端の「Download」リンクをクリックする。
   - 画面上部の「Python 2.7.2 Windows Installer」リンク(2011年12月 執筆時点の最新版)をクリックする。
   - ダウンロード画面が表示されるため、「OK」ボタンを押す。

  (2) Pythonのインストール
   - ダウンロードした「python-2.7.2.msi」ファイルをダブルクリックする。
   - 指示に従ってインストールを行う。なお、インストール画面は以下の通り。

.. warning::

   上記のインストーラは32bit版であり、64bit版を利用している場合は「Python 2.7.2 Windows X86-64 Installer 」リンクをクリックすること。

.. figure:: images/doc_python_install-1.*
.. figure:: images/doc_python_install-2.*
.. figure:: images/doc_python_install-3.*
.. figure:: images/doc_python_install-4.*

 1.2 easy_installのインストール
  easy_installとは、パッケージ管理システムからPythonのモジュールを自動で検索し、インストールやアップデートをするツールである。
  Sphinxはこのツールを利用してインストールする。
  easy_installのインストール手順を示す。なお、既にインストールされている場合は、本手順は不要である。

  (1) easy_installファイルのダウンロード
   - Internet Explorer等のWebブラウザを利用し、http://peak.telecommunity.com/dist/ez_setup.py の画面を開く。
   - 表示された画面上で右クリックをし、「名前を付けてページを保存」を実行する。なお、その際に保存するファイル名は「ez_setup.py」とし、Cドライブ直下に保存する。

  (2) easy_installのインストール
   - コマンドプロンプト画面を開く。（コマンドプロンプト画面は、スタート->プログラム->アクセサリ->コマンドプロンプト の手順で表示することができる）
   - コマンドプロンプト画面からCドライブ直下に移動する。(コマンドプロンプト画面で「cd C:\」を入力後、Enterを押すことでCドライブ直下に移動できる)
   - コマンドプロンプト画面で「python ez_setup.py」を入力後、Enterを押す。

.. figure:: images/doc_easy_install_install.*

  easy_installのインストール画面

 1.3 Sphinxのインストール
  Sphinxのインストール手順を示す。

  (1) Sphinxのインストール
   - コマンドプロンプト画面を開く。
   - コマンドプロンプト画面で「easy_install sphinx」を入力後、Enterを押す。

.. figure:: images/doc_sphinx_install.*

  Sphinxのインストール画面

 2. Sphinxのインストール(Linux編)

.. todo:: Sphinxのインストール(Linux編)の記述を追加する

 3. Sphinxのプロジェクト作成
  Sphinxではプロジェクトという単位で関連ドキュメントを作成する。
  プロジェクトを作成する手順は以下の通り。
   - sphinx-quickstartの実行
   - ページ構成の作成

  なお、プロジェクト情報は以下として作成する。

.. csv-table:: Frozen Delights!
   :header: "項目", "内容"
   :widths: 20, 20

   "プロジェクトの作成場所","C:\sample-project"
   "プロジェクト名","sample-project"
   "バージョン番号","2011.01.01"
   "著者の名前","sample"

 3.1 sphinx-quickstartの実行
  sphinx-quickstartとは、Sphinxのプロジェクトを作成するコマンドである。実行手順を以下に示す。

  (1) sphinx-quickstartの実行

   - コマンドプロンプト画面を開く。
   - コマンドプロンプト画面で「mkdir C:\sample-project」を入力後、Enterを押し、プロジェクトフォルダを作成する。
   - コマンドプロンプト画面からC:\sample-project直下に移動する。(コマンドプロンプト画面で「cd C:\sample-project」を入力後、Enterを押すことで移動できる)
   - コマンドプロンプト画面で「sphinx-quickstart」を入力後、Enterを押し、プロジェクト情報を入力する。なお、以降の★で示す、「プロジェクト名」、「バージョン番号」、「著者の名前」以外はデフォルトでも特に問題ない。詳細は 「Sphinxの日本ユーザ会」のページを参照。http://sphinx-users.jp/gettingstarted/sphinxquickstart.html。

   - Internet Explorer等のWebブラウザを利用し、http://python.org の画面を開く。
   - 画面左端の「Download」リンクをクリックする。
   - 画面上部の「Python 2.7.2 Windows Installer」リンク(2011年12月 執筆時点の最新版)をクリックする。
   - ダウンロード画面が表示されるため、「OK」ボタンを押す。

::

  C:\sample-project>sphinx-quickstart
  Welcome to the Sphinx 1.1 quickstart utility.
  
  Please enter values for the following settings (just press Enter to
  accept a default value, if one is given in brackets).
  
  Enter the root path for documentation.
  > Root path for the documentation [.]:
  
  You have two options for placing the build directory for Sphinx output.
  Either, you use a directory "_build" within the root path, or you separate
  "source" and "build" directories within the root path.
  > Separate source and build directories (y/N) [n]:
  
  Inside the root directory, two more directories will be created; "_templates"
  for custom HTML templates and "_static" for custom stylesheets and other static
  files. You can enter another prefix (such as ".") to replace the underscore.
  > Name prefix for templates and static dir [_]:
  
  The project name will occur in several places in the built documentation.
  > Project name: sample-project  <--- ★プロジェクト名
  > Author name(s): sample        <--- ★著者の名前
  
  Sphinx has the notion of a "version" and a "release" for the
  software. Each version can have multiple releases. For example, for
  Python the version is something like 2.5 or 3.0, while the release is
  something like 2.5.1 or 3.0a1.  If you don't need this dual structure,
  just set both to the same value.
  > Project version: 2012.01.01   <--- ★バージョン番号
  > Project release [2012.01.01]:
  
  The file name suffix for source files. Commonly, this is either ".txt"
  or ".rst".  Only files with this suffix are considered documents.
  > Source file suffix [.rst]:
  
  One document is special in that it is considered the top node of the
  "contents tree", that is, it is the root of the hierarchical structure
  of the documents. Normally, this is "index", but if your "index"
  document is a custom template, you can also set this to another filename.
  > Name of your master document (without suffix) [index]:
  
  Sphinx can also add configuration for epub output:
  > Do you want to use the epub builder (y/N) [n]:
  
  Please indicate if you want to use one of the following Sphinx extensions:
  > autodoc: automatically insert docstrings from modules (y/N) [n]:
  > doctest: automatically test code snippets in doctest blocks (y/N) [n]:
  > intersphinx: link between Sphinx documentation of different projects (y/N) [n]:
  > todo: write "todo" entries that can be shown or hidden on build (y/N) [n]:
  > coverage: checks for documentation coverage (y/N) [n]:
  > pngmath: include math, rendered as PNG images (y/N) [n]:
  > mathjax: include math, rendered in the browser by MathJax (y/N) [n]:
  > ifconfig: conditional inclusion of content based on config values (y/N) [n]:
  > viewcode: include links to the source code of documented Python objects (y/N) [n]:
  
  A Makefile and a Windows command file can be generated for you so that you
  only have to run e.g. `make html' instead of invoking sphinx-build
  directly.
  > Create Makefile? (Y/n) [y]:
  > Create Windows command file? (Y/n) [y]:
  
  Creating file .\conf.py.
  Creating file .\index.rst.
  Creating file .\Makefile.
  Creating file .\make.bat.
  
  Finished: An initial directory structure has been created.
  
  You should now populate your master file .\index.rst and create other documentation
  source files. Use the Makefile to build the docs, like so:
     make builder
  where "builder" is one of the supported builders, e.g. html, latex or linkcheck.
  
  
  C:\sample-project>

 3.2 ページ構成の作成
  sphinx-quickstartで作成したプロジェクト内にドキュメントを作成する。
  なお、ページ構成は以下とする。

::

  index.rst
    +- sample1.rst
    +- sample2.rst

  (1) rstファイルの作成
   - C:\sample-project直下にindex.rst、sample1.rst、sample2.rstファイルを作成する。

**index.rst**

.. code-block:: rst
   :linenos:

   ロボット開発環境
   ----------------

   Contents:

   .. toctree::
      :maxdepth: 2

      sample1
      sample2

**sample1.rst**

.. code-block:: rst
   :linenos:

   ==================================
   ドキュメンテーションフレームワーク
   ==================================

**sample2.rst**

.. code-block:: rst
   :linenos:

   ==========================
   テスティングフレームワーク
   ==========================

  (2) htmlファイルの作成
   - コマンドプロンプト画面を開き、C:\sample-projectに移動する。
   - コマンドプロンプト画面に「make html」を入力後、Enterを押し、htmlファイルを作成する。

テスティングフレームワーク
==========================

テストの必要性
--------------

 - プログラムには必ず変更がある
 - プログラムには通常バグも含まれる
 - リファクタリングが登場する以前は、一度正常な動作をしたプログラムは二度と手を触れるべきではないと言われていた。下手に手を加えて動作が変わってしまうと、それに伴って関連する部分にも修正が加えられ、やがて修正はプロジェクト全体に波及し対処しきれなくなるかも知れない
 - 

.. todo:: テストの必要性を記述する


 * 

- 以下の文書を記述

 http://gihyo.jp/dev/feature/01/hudson/0001

Continuous Integration，以下CI）のおさらいをしましょう。CIは，Extreme Programmingに端を発し，
Martin Fowlerによって広められた概念で，狭義には，別々に開発された部品を持ち寄ってお互いの動作を検証
する「統合テスト」を早い段階から恒常的に行うことを指します。この当初の概念には必ずしも統合テストの
自動化という考え方は含まれていませんでしたが，最近では，CIは単に統合テストだけではなく，広くビルド
及びテスト全般を恒常的に行うことを指すようになり，またこれを現実的な工数で実現するための必須の手段
として，ビルド・テストの工程を極力自動化する，という事が重要なポイントの一つになってきました。

この考え方の背景の一つには，コンピュータの高性能化・低価格化する一方，人件費はむしろ高くなっている
という経済的な現実があります。この流れの結果，今日では，技術者の生産性の向上に少しでも寄与するならば
コンピュータを湯水のように無駄遣いしても元が取れる，ということになってきました。こういう考え方に立
てば，ソースコード管理システムに投入される変更一つ一つに対してビルドとテストを行って変更の質を確認する，
というプロセスも決しておかしくはない事になります。

もちろん，最終的な目的は技術者の生産性を向上させる，つまり我々エンジニアが楽をする，という事なわけ
ですから，CIを導入するのに手間が掛かるようでは本末転倒です。この点について，CruiseControlを始めと
する初期のツール群には色々な問題がありましたが，ここ数年の間に登場した第二世代のCI ツールによって，
CIは開発の現場で実用可能なレベルに到達してきました。

**Ｔ．Ｂ．Ｄ**

Jenkins
-------

- 以下の文書を記述

 https://wiki.jenkins-ci.org/display/JA/Meet+Jenkins


概要
~~~~

 Jenkinsは、ソフトウェアのビルドやcronで起動するジョブなどの繰り返しのジョブの実行を監視します。
 これらのうち、Jenkinsは現在次の2つのジョブに重点を置いています。

 1 継続的な、ソフトウェアプロジェクトのビルドとテスト: つまり、CruiseControlやDamageControlが行うこと。
  一言で言えば、Jenkinsは、容易ないわゆる「継続インテグレーションシステム」を提供し、開発者が変更を
  プロジェクトに統合でき、ユーザーがより新しいビルドを容易に取得できるようにします。自動化された
  継続的なビルドは、生産性を向上させます。

 2 外部で起動するジョブの実行監視: cronによるジョブやprocmailのジョブで、リモートマシンで動作するも
 のも含みます。例えばcronについて言えば、出力をキャプチャーした定期的なメールだけ受信し、こつこつと
 それを見ます。おかしくなっていることに気がつくかどうかは、すべてあなた次第です。Jenkinsは出力を
 保存し、 いつおかしくなったのか容易に把握することができるようになります。

*特徴*

 1 簡易なインストール: java -jar jenkins.war　を実行するか、サーブレットコンテナにデプロイします。
   追加のインストールも、データベースも不要です。

 2 簡易な設定: 豊富な入力時のエラーチェックとヘルプを備えたわかりやすいWebGUIを使用して、Jenkinsを
   設定できます。もう手でXMLをいじる必要はありません。いじりたいのならそうすることもできますが。

 3 差分のサポート: Jenkinsは、CVSやSubversionからビルドへの変更の一覧を生成することができます。
   これは、リポジトリの負荷を削減するとても効率的な方法で行われます。

 4 永続リンク: どこからでも簡単にリンクできるように、"最新のビルド"や"最新の安定ビルド"のような永続
   (固定)リンクを含む、多くの画面は、クリーンでわかりやすいURLを持ちます。

 5 RSS/Eメール/IM との連携: 失敗時にリアルタイムに通知をうけるために、RSSやEメールでビルド結果を
   監視します。

 6 ビルド後のタグ: ビルドが完了した後に、ビルドにタグを付与できます。

 7 JUnit/TestNGによるテスト結果のレポート: JUnitのテスト結果を、一覧表示および要約し、いつから失敗
   しているのかなどの履歴情報とともに表示します。履歴の傾向はグラフ化されます。

 8 分散ビルド: Jenkinsは、複数のコンピュータで分散ビルド/テストを実行できます。このおかげで、開発者
   の机の下に横たわっている何もしていないワークステーションを利用することができます。

 9 ファイル指紋: Jenkinsは、どのビルドがどのjarを生成したのか、どのビルドがjarのどのバージョンを使用
   しているのか等々、追跡できます。この機能は、Jenkinsが管理しないjarでも機能します。そして、
   プロジェクトの依存性を管理するのにも有用です。

 10 プラグインサポート: Jenkinsをサードパーティのプラグインで拡張できます。 開発チームが使用するツール
    や処理をサポートするプラグインを書くこともできます。


導入
~~~~

- 以下の文書を記述

 https://wiki.jenkins-ci.org/display/JENKINS/Installing+Jenkins+on+Ubuntu

 1 インストール

 ::
 
   wget -q -O - http://pkg.jenkins-ci.org/debian/jenkins-ci.org.key | sudo apt-key add -
   sudo sh -c 'echo deb http://pkg.jenkins-ci.org/debian binary/ > /etc/apt/sources.list.d/jenkins.list'
   sudo aptitude update
   sudo aptitude install jenkins

 ※ apacheの記述も必要

ソースコードリポジトリ
======================

ソースコードのバージョン管理
----------------------------

- 以下の文書を記述

 http://ja.wikipedia.org/wiki/バージョン管理システム

バージョン管理システム（バージョンかんりシステム）とは、コンピュータ上で作成、編集されるファイルの
変更履歴を管理するためのシステム。特にソフトウェア開発においてソースコードの管理に用いられることが多い。
バージョン管理システムの最も基本的な機能は、ファイルの作成日時、変更日時、変更点などの履歴を保管するこ
とである。これにより、何度も変更を加えたファイルであっても、過去の状態や変更内容を確認したり、変更前の
状態を復元することが容易になる。更に、多くのバージョン管理システムでは、複数の人間がファイルの編集に関
わる状況を想定している。商業的なソフトウェア開発やオープンソースプロジェクトなどでは、複数の人間が複数
のファイルを各々編集するため、それぞれのファイルの最新の状態が分からなくなったり、同一ファイルに対する
変更が競合するなどの問題が生じやすいが、バージョン管理システムは、このような問題を解決する仕組みを提供
する。ただし、バージョン管理システムを個人のファイル管理に使用することも可能であるし、ソフトウェアの
ソースコードだけでなく、設定ファイルや原稿の管理などにも使うことも可能である。

Subversion
----------

- 以下の文書を記述

 http://ja.wikipedia.org/wiki/Subversion

 http://subversion.apache.org/packages.html


Subversion（サブバージョン、サバージョン; SVN）はプログラムのソースコードなどを管理する集中型
バージョン管理システムの一つ。2009年11月7日にApache Incubatorプロジェクトのひとつとなり、
2010年2月17日よりApacheのトッププロジェクトとなった。ライセンスはApache Licenseに準じたものとなっている。
歴史的には広く使われているバージョン管理システムの一つにCVSがあった。CVSはよくできているが、
ディレクトリの移動の管理やネットワーク対応の点、不可分な更新などでやや難があった。これらCVSの問題点を
解決すべく開発されたのがSubversionである。 古くからオープンソースソフトウェアの開発においてはCVSが多く
使われていたが、近年ではSubversionを使用するオープンソースプロジェクトも多くなりつつある。
Subversionは集中型（クライアント・サーバ型）であるが、その後、GitやMercurialやBazaarなどの分散型の
バージョン管理システムが登場するようになった。例えば、Linuxカーネルの管理にはGit、Mozilla Firefoxの
管理にはMercurial、MySQLの管理にはBazaarが使われている。


Sourceforge
-----------

- 以下の文書を記述

 http://ja.wikipedia.org/wiki/SourceForge.JP
 

SourceForge.JP（ソースフォージドットジェーピー）は、日本のオープンソースソフトウェアプロジェクト向け
のホスティングサイト。SourceForge.netの姉妹サイトで、OSDN社が運営している。
SourceForge.netの日本語版サイトとして、VA Linux Systems JapanのOSDN事業部によって2002年に設立
（2002年3月ベータ公開、2002年4月正式運用開始）。現在は2007年9月にVA LinuxからスピンオフしたOSDN株式
会社によって運営されている。提供されているサービスはSourceForge.netとかぶる部分が多いが、コンパイル
ファームのようにSourceForge.JPにしかないサービスもある（詳細はサービスの節を参照）。

SourceForge.JPではホスティング費用は発生しないが、オープンソースプロジェクトホスティングサイトなので、
開発成果はオープンソースとして公開する必要がある。ライセンスはOSIにオープンソースライセンスとして承認
されているものが利用可能。

企業によるオープンソース活動の拠点としても利用されており、登録開発者には個人のほか、それらの企業に所属
する開発者も多い。

2008年8月現在の登録プロジェクト数は3,263、登録ユーザ数は30,035。

Git
---

- 以下の文書を記述

 http://ja.wikipedia.org/wiki/Git

 http://git-scm.com/download
 

Git（ギット）はプログラムなどのソースコード管理を行う分散型バージョン管理システム。動作速度に重点が
置かれている。Linuxカーネルのソースコード管理を目的として、リーナス・トーバルズによって開発された。
現在のメンテナンスは濱野純 (Junio C Hamano)が担当している。

Gitではワーキングディレクトリがリポジトリの全ての履歴を含んでいるため、中央サーバへのアクセスが不可能
な状態であってもリビジョン間の履歴を調査することができる。

Linuxカーネルの開発では、Linux Kernel Mailing Listに投稿される多数のパッチをメンテナーたちがソース
コードに適用するという形式を採用している。これらの作業を効率的にするため、当初BitKeeperというバージョン
管理システムを用いていたが、このソフトウェアは商用ソフトウェア（クライアントはバイナリのみ無料で、
サーバは商用だがBitMover社の好意で無料で使えていた）であった。この状況を快く思わない人々がBitKeeperの
クローンを実装したことからこの環境が使えなくなってしまい（BitKeeper#ライセンス問題やBitKeeper#価格変
更を参照）、その代替として2005年にGitが開発された。

Linuxのカーネルでは、相当量のソースコードを扱うため、変更点の抽出やリポジトリ操作に時間がかかっていて
は困るという状況になっていた。他の様々なバージョン管理システムをあたったが十分なものがなかった。 
そのため、このような問題もできるかぎり解決できるよう、いくつかの案が導入されている（この部分は、
他のバージョン管理システムにも同様の機能が導入されるようになった）。


