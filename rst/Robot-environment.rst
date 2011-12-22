ドキュメンテーションフレームワーク
==================================

ドキュメントの必要性
--------------------

ソフトウェア開発には以下のような特徴がある．

* ソフトウェア開発は，「設計」→「実装」→「試験」の手順で行う．
* ソフトウェア開発は，作成するソフトウェアの規模，期間に応じ，1人から複数人で行う．
* 一度作成したソフトウェアに対する不具合の対応，機能追加を行うことがある．機能追加前後の開発者が異なることもある．

よって，以下の理由により，ドキュメントが必要である．

* 複数の人で開発を行う場合は，ソフトウェアの設計情報を共有する必要があるため．
* 一度作成したソフトウェアに対する不具合の対応，機能追加を行うような場合，どのような構成でソフトウェアが作成されているかを確認する必要があるため．

ここで，ドキュメントの作成には，従来Microsoft OfficeのWordなどの専用のソフトウェアを利用することが多かったと思われる．専用のソフトウェアの場合，ソフトウェアがインストールされた端末でしかドキュメントが作成できない，GUIの作業環境が必要であるなどの問題点があった．

これらの問題を解決する方法の一つとして，現在注目を浴びているreStructruedTextというマークアップ言語，Sphinxというソフトウェアを以降の章で紹介する．

reStructuredText / Sphinx
--------------------------

概要
^^^^

reStructuredTextとは，文書の構造や表示の仕方などを定義したマークアップ言語のひとつである．
reStructuredText は RST，ReST，reSTと略されることもある．
reStructuredTextの特徴は以下の通り．

* テキスト形式で記述するため，OSに標準でインストールされているメモ帳などのテキストエディタで作成することができる．(専用のソフトウェアのインストールは不要)
* ソースコードの状態でも高い可読性を持っている．
* reStructuredTextで記述された文書はPDF，HTML，XML，LaTeXなど複数の形式の文書に変換することができる．

Sphinxはドキュメントを簡単に作れるようにするためのソフトウェアである．
SphinxはreStructuredTextをマークアップ言語として使用しており，作成したドキュメントをHTML，PDFなどの形式に変換することができる．
SphinxはreStructuredTextの特徴は以下の通り．

* 実行環境はWindows/Linux/Mac OS，Python 2.4以上(TODO: 再度確認)のインストールが必要である．
* 出力フォーマットはPDF，HTML，XML，LaTeX，ePubなどをサポートしている．
* 引用，用語解説，ソースコードへのクロスリファレンス機能．
* 引用したソースコードを自動でハイライトする機能．
* 章などのインデックスを自動で設定する機能．

導入
^^^^

SphinxはPythonを利用して作成されていることから，SphinxのインストールにはPython，Sphinxの両方が必要となる．Sphinxの導入手順(Windows編，Linux編)を以降に示す．

`Windows編`

WindowsにSphinxを導入する手順は以下の通り．

* Pythonのインストール
* easy_installのインストール
* Sphinxのインストール
* Sphinxプロジェクトの作成

1 *Pythonのインストール*
  
Pythonのインストール手順を示す．なお，既にインストールされている場合は，本手順は不要である．

\(1\) Pythonのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://python.org の画面を開く．
* 画面左端の「Download」リンクをクリックする．
* 画面上部の「Python 2.7.2 Windows Installer」リンク(2011年12月 執筆時点の最新版)をクリックする．
* ダウンロード画面が表示されるため，「OK」ボタンを押す．

  .. warning::

     上記のインストーラは32bit版であり，64bit版を利用している場合は「Python 2.7.2 Windows X86-64 Installer 」リンクをクリックすること．

\(2\) Pythonのインストール

* ダウンロードした「python-2.7.2.msi」ファイルをダブルクリックする．
* 指示に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_python_install-1.*

  Pythonのインストール画面(1/4)

.. figure:: images/doc_python_install-2.*

  Pythonのインストール画面(2/4)

.. figure:: images/doc_python_install-3.*

  Pythonのインストール画面(3/4)

.. figure:: images/doc_python_install-4.*

  Pythonのインストール画面(4/4)

..

2 *easy_installのインストール*

easy_installとは，パッケージ管理システムからPythonのモジュールを自動で検索し，インストールやアップデートをするツールである．
Sphinxはこのツールを利用してインストールする．
easy_installのインストール手順を示す．なお，既にインストールされている場合は，本手順は不要である．

\(1\) easy_installファイルのダウンロード

 * Internet Explorer等のWebブラウザを利用し，http://peak.telecommunity.com/dist/ez_setup.py の画面を開く．
 * 表示された画面上で右クリックをし，「名前を付けてページを保存」を実行する．なお，その際に保存するファイル名は「ez_setup.py」とし，Cドライブ直下に保存する．

\(2\) easy_installのインストール

 * コマンドプロンプト画面を開く．（コマンドプロンプト画面は，スタート->プログラム->アクセサリ->コマンドプロンプト の手順で表示することができる）
 * コマンドプロンプト画面からCドライブ直下に移動する．(コマンドプロンプト画面で「cd C:\\」を入力後，Enterを押すことでCドライブ直下に移動できる)
 * コマンドプロンプト画面で「python ez_setup.py」を入力後，Enterを押す．

..

   .. figure:: images/doc_easy_install_install.*
  
     easy_installのインストール画面

..


3 *Sphinxのインストール*

Sphinxのインストール手順を示す．

\(1\) Sphinxのインストール

* コマンドプロンプト画面を開く．
* コマンドプロンプト画面で「easy_install sphinx」を入力後，Enterを押す．

.. figure:: images/doc_sphinx_install.*

  Sphinxのインストール画面

..


4 *Sphinxのプロジェクト作成*

Sphinxではプロジェクトという単位で関連ドキュメントを作成する．プロジェクトを作成する手順は以下の通り．

* sphinx-quickstartの実行
* ページ構成の作成

なお，プロジェクト情報は以下として作成する．

      .. csv-table:: Sphinxのプロジェクト情報(Windows編)
         :header: "項目", "内容"
         :widths: 20, 20

         "プロジェクトの作成場所","C:\\sample-project"
         "プロジェクト名","sample-project"
         "バージョン番号","2012.01.01"

..
..

4.1 *sphinx-quickstartの実行*

sphinx-quickstartとは，Sphinxのプロジェクトを作成するコマンドである．実行手順を以下に示す．

\(1\) sphinx-quickstartの実行

* sphinx-quickstartの実行
* ページ構成の作成
* コマンドプロンプト画面を開く．
* コマンドプロンプト画面で「mkdir C:\\sample-project」を入力後，Enterを押し，プロジェクトフォルダを作成する．
* コマンドプロンプト画面からC:\\sample-project直下に移動する．(コマンドプロンプト画面で「cd C:\\sample-project」を入力後，Enterを押すことで移動できる)
* コマンドプロンプト画面で「sphinx-quickstart」を入力後，Enterを押し，プロジェクト情報を入力する．なお，以降の★で示す，「プロジェクト名」，「バージョン番号」，「著者の名前」以外はデフォルトでも特に問題ない．詳細は 「Sphinxの日本ユーザ会」のページを参照．http://sphinx-users.jp/gettingstarted/sphinxquickstart.html．

   .. code-block:: bash
     :linenos:
   
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
   
     Inside the root directory, two more directories will be created; 
     "_templates"
     for custom HTML templates and "_static" for custom stylesheets and other 
     static files. You can enter another prefix (such as ".") to replace 
     the underscore.
     > Name prefix for templates and static dir [_]:
   
     The project name will occur in several places in the built documentation.
     > Project name: sample-project  <--- ★プロジェクト名
     > Author name(s): sample        <--- ★著者の名前
   
     Sphinx has the notion of a "version" and a "release" for the
     software. Each version can have multiple releases. For example, for
     Python the version is something like 2.5 or 3.0, while the release is
     something like 2.5.1 or 3.0a1.  If you do not need this dual structure,
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
     > intersphinx: link between 
     > Sphinx documentation of different projects (y/N) [n]:
     > todo: write "todo" entries that can be shown or hidden on build (y/N) [n]:
     > coverage: checks for documentation coverage (y/N) [n]:
     > pngmath: include math, rendered as PNG images (y/N) [n]:
     > mathjax: include math, rendered in the browser by MathJax (y/N) [n]:
     > ifconfig: conditional inclusion of content based on 
     > config values (y/N) [n]:
     > viewcode: include links to the source code of documented 
     > Python objects (y/N) [n]:
   
     A Makefile and a Windows command file can be generated for you so that you
     only have to run e.g. make html instead of invoking sphinx-build
     directly.
     > Create Makefile? (Y/n) [y]:
     > Create Windows command file? (Y/n) [y]:
   
     Creating file .\conf.py.
     Creating file .\index.rst.
     Creating file .\Makefile.
     Creating file .\make.bat.
   
     Finished: An initial directory structure has been created.
   
     You should now populate your master file .\index.rst and create other 
     documentation source files. Use the Makefile to build the docs, like so:
        make builder
     where "builder" is one of the supported builders, 
     e.g. html, latex or linkcheck.
   
   
     C:\sample-project>

4.2 *ページ構成の作成*

sphinx-quickstartで作成したプロジェクト内にドキュメントを作成する．
なお，ページ構成は以下とする．

  ::

    index.rst
      +- sample1.rst
      +- sample2.rst

\(1\) rstファイルの作成

* C:\\sample-project直下にindex.rst，sample1.rst，sample2.rstファイルを作成する．

     *index.rst*

     .. code-block:: rst
        :linenos:

        ロボット開発環境
        ----------------

        Contents:

        .. toctree::
           :maxdepth: 2

           sample1
           sample2

     *sample1.rst*

     .. code-block:: rst
        :linenos:

        ==================================
        ドキュメンテーションフレームワーク
        ==================================

     *sample2.rst*

     .. code-block:: rst
        :linenos:

        ==========================
        テスティングフレームワーク
        ==========================

..

\(2\) htmlファイルの作成

* コマンドプロンプト画面を開き，C:\\sample-projectに移動する．
* コマンドプロンプト画面に「make html」を入力後，Enterを押し，htmlファイルを作成する．(C:\\sample-project\\_build\\html\\index.htmlがある)


`Linux編`

Linux(Ubuntu)にSphinxをインストールする方法は，「パッケージシステムを利用したインストール」と「手動インストール」の2つある．
「手動インストール」については，Sphinxのインストール(Windows編)の「2 easy_installのインストール」，「3 Sphinxのインストール」と同様である．
以降には，「パッケージシステムを利用したインストール」手順を示す．

1 *パッケージシステムを利用したインストール*

ターミナル画面から以下のコマンドを実行する．

  .. code-block:: bash

    aptitude install python-sphinx

上記コマンドの実行結果の内容は以下の通り．

.. 以下コメントアウト
   testUser@testUser-desktop:~$ sudo aptitude install python-sphinx
   パッケージリストを読み込んでいます... 完了
   依存関係ツリーを作成しています
   状態情報を読み取っています... 完了
   Reading extended state information
   Initializing package states... 完了
   Writing extended state information... 完了
   The following NEW packages will be installed:
     python-docutils{a} python-jinja2{a} python-lxml{a} python-pygments{a} python-roman{a} python-sphinx
   0 packages upgraded, 6 newly installed, 0 to remove and 201 not upgraded.
   Need to get 3,359kB of archives. After unpacking 14.1MB will be used.
   Do you want to continue? [Y/n/?]
   WARNING: untrusted versions of the following packages will be installed!
   
   Untrusted packages could compromise your systems security.
   You should only proceed with the installation if you are certain that
   this is what you want to do.
   
     python-sphinx python-pygments python-lxml python-docutils python-roman python-jinja2
   
   Do you want to ignore this warning and proceed anyway?
   To continue, enter "Yes"; to abort, enter "No": Yes
   Writing extended state information... 完了
   Get:1 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-roman 0.6-3 [15.1kB]
   Get:2 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-docutils 0.6-3 [1,703kB]
   Get:3 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-jinja2 2.3.1-1 [160kB]
   Get:4 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-lxml 2.2.4-1 [651kB]
   Get:5 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-pygments 1.2.2+dfsg-1ubuntu1 [325kB]
   Get:6 http://jp.archive.ubuntu.com/ubuntu/ lucid/main python-sphinx 0.6.4-1 [506kB]
   Fetched 3,359kB in 6s (513kB/s)
   未選択パッケージ python-roman を選択しています．
   (データベースを読み込んでいます ... 現在 122421 個のファイルとディレクトリがインストールされています．)
   (.../python-roman_0.6-3_all.deb から) python-roman を展開しています...
   未選択パッケージ python-docutils を選択しています．
   (.../python-docutils_0.6-3_all.deb から) python-docutils を展開しています...
   未選択パッケージ python-jinja2 を選択しています．
   (.../python-jinja2_2.3.1-1_amd64.deb から) python-jinja2 を展開しています...
   未選択パッケージ python-lxml を選択しています．
   (.../python-lxml_2.2.4-1_amd64.deb から) python-lxml を展開しています...
   未選択パッケージ python-pygments を選択しています．
   (.../python-pygments_1.2.2+dfsg-1ubuntu1_all.deb から) python-pygments を展開しています...
   未選択パッケージ python-sphinx を選択しています．
   (.../python-sphinx_0.6.4-1_all.deb から) python-sphinx を展開しています...
   doc-base のトリガを処理しています ...
   Processing 26 changed 2 added doc-base file(s)...
   Registering documents with scrollkeeper...
   man-db のトリガを処理しています ...
   python-roman (0.6-3) を設定しています ...
   
   python-docutils (0.6-3) を設定しています ...
   
   python-jinja2 (2.3.1-1) を設定しています ...
   
   python-lxml (2.2.4-1) を設定しています ...
   
   python-pygments (1.2.2+dfsg-1ubuntu1) を設定しています ...
   
   python-sphinx (0.6.4-1) を設定しています ...
   
   python-support のトリガを処理しています ...
   python-central のトリガを処理しています ...
   パッケージリストを読み込んでいます... 完了
   依存関係ツリーを作成しています
   状態情報を読み取っています... 完了
   Reading extended state information
   Initializing package states... 完了
   Writing extended state information... 完了
   
   testUser@testUser-desktop:~$

..

2 *Sphinxのプロジェクト作成*

Sphinxではプロジェクトという単位で関連ドキュメントを作成する．プロジェクトを作成する手順は以下の通り．

* sphinx-quickstartの実行
* ページ構成の作成

なお，プロジェクト情報は以下として作成する．

      .. csv-table:: Sphinxのプロジェクト情報(Linux編)
         :header: "項目", "内容"
         :widths: 20, 20

         "プロジェクトの作成場所","/home/testUser/sample-project"
         "プロジェクト名","sample-project"
         "バージョン番号","2012.01.01"
         "著者の名前","sample"

2.1 *sphinx-quickstartの実行*

sphinx-quickstartとは，Sphinxのプロジェクトを作成するコマンドである．実行手順を以下に示す．

\(1\) sphinx-quickstartの実行

* ターミナル画面を開く．
* ターミナル画面でtestUserユーザのホームディレクトリ(/home/testUser)に移動し，ホームディレクトリ直下にsample-projectディレクトリを作成する．(mkdir sample-project)
* ターミナル画面で「sphinx-quickstart」を入力後，Enterを押し，プロジェクト情報を入力する．なお，以降の★で示す，「プロジェクト名」，「バージョン番号」，「著者の名前」以外はデフォルトでも特に問題ない．詳細は 「Sphinxの日本ユーザ会」のページを参照．http://sphinx-users.jp/gettingstarted/sphinxquickstart.html．

   .. code-block:: bash
     :linenos:
   
     testUser@testUser-desktop:~/sample-project$ sphinx-quickstart
     Welcome to the Sphinx quickstart utility.
   
     Please enter values for the following settings (just press Enter to
     accept a default value, if one is given in brackets).
   
     Enter the root path for documentation.
     > Root path for the documentation [.]:
   
     You have two options for placing the build directory for Sphinx output.
     Either, you use a directory "_build" within the root path, or you separate
     "source" and "build" directories within the root path.
     > Separate source and build directories (y/N) [n]:
   
     Inside the root directory, 
     two more directories will be created; "_templates"
     for custom HTML templates and "_static" for custom stylesheets 
     and other static files. You can enter another prefix (such as ".") 
     to replace the underscore.
     > Name prefix for templates and static dir [_]:
   
     The project name will occur in several places in the built documentation.
     > Project name: sample-project
     > Author name(s): sample
   
     Sphinx has the notion of a "version" and a "release" for the
     software. Each version can have multiple releases. For example, for
     Python the version is something like 2.5 or 3.0, while the release is
     something like 2.5.1 or 3.0a1.  If you do not need this dual structure,
     just set both to the same value.
     > Project version: 2012.01.01
     > Project release [2012.01.01]:
   
     The file name suffix for source files. Commonly, this is either ".txt"
     or ".rst".  Only files with this suffix are considered documents.
     > Source file suffix [.rst]:
   
   
     One document is special in that it is considered the top node of the
     "contents tree", that is, it is the root of the hierarchical structure
     of the documents. Normally, this is "index", but if your "index"
     document is a custom template, you can also set this to another filename.
     > Name of your master document (without suffix) [index]:
   
     Please indicate if you want to use one of the following Sphinx extensions:
     > autodoc: automatically insert docstrings from modules (y/N) [n]:
     > doctest: automatically test code snippets in doctest blocks (y/N) [n]:
     > intersphinx: link between Sphinx documentation 
     > of different projects (y/N) [n]:
     > todo: write "todo" entries that can be shown or hidden on build (y/N) [n]:
     > coverage: checks for documentation coverage (y/N) [n]:
     > pngmath: include math, rendered as PNG images (y/N) [n]:
     > jsmath: include math, rendered in the browser by JSMath (y/N) [n]:
     > ifconfig: conditional inclusion of content based on 
     > config values (y/N) [n]:
   
     A Makefile and a Windows command file can be generated for you so that you
     only have to run e.g. make html instead of invoking sphinx-build
     directly.
     > Create Makefile? (Y/n) [y]:
     > Create Windows command file? (Y/n) [y]:
   
     Finished: An initial directory structure has been created.
   
     You should now populate your master file ./index.rst and 
     create other documentation
     source files. Use the Makefile to build the docs, like so:
        make builder
     where "builder" is one of the supported builders, 
     e.g. html, latex or linkcheck.
   
     testUser@testUser-desktop:~/sample-project$

2.2 *ページ構成の作成*

sphinx-quickstartで作成したプロジェクト内にドキュメントを作成する．
なお，ページ構成は以下とする．

  ::

    index.rst
      +- sample1.rst
      +- sample2.rst

\(1\) rstファイルの作成

* /home/testUser/sample-project直下にindex.rst，sample1.rst，sample2.rstファイルを作成する．

       *index.rst*

       .. code-block:: rst
          :linenos:

          ロボット開発環境
          ----------------

          Contents:

          .. toctree::
             :maxdepth: 2

             sample1
             sample2

       *sample1.rst*

       .. code-block:: rst
          :linenos:

          ==================================
          ドキュメンテーションフレームワーク
          ==================================

       *sample2.rst*

       .. code-block:: rst
          :linenos:

          ==========================
          テスティングフレームワーク
          ==========================

\(2\) htmlファイルの作成

* ターミナル画面を開き，/home/testUser/sample-projectディレクトリに移動する．
* ターミナル画面で「make html」を入力後，Enterを押し，htmlファイルを作成する．(/home/testUser/sample-project/_build/html/index.htmlがある)


ソースコードリポジトリ
======================

ソースコードのバージョン管理
----------------------------

ソフトウェアの開発では日常的にファイルの追加，修正を行うため，定期的にバックアップを取ることが重要である．バックアップをとる場合，通常ファイル名やフォルダ名に日付などを追加するが，この方法には以下のような問題がある．

* 前回のバックアップからの変更点がわからない．(変更履歴の問題)
* 毎回全てのデータを保存することになるため，ディスク容量を必要以上に使用してしまう．(ディスク容量の問題)

上記の問題を解決するためのシステムをバージョン管理システムと呼び，現在のソフトウェア開発では日常的に利用されている．バージョン管理システムには以下のような特徴がある．

* ファイルの変更履歴を管理し，変更履歴から変更点の比較が行える．また，過去のファイルを取り出すこともできる．誤って削除してしまっても元に戻すことができる．
* ファイルの変更点の管理は，通常前回データの差分のみであり，ディスク容量を必要以上に使用しない．
* 多くのバージョン管理システムは複数人の利用を想定しており，複数の人が同時に同一のファイルを修正した場合の問題を解決する仕組みを提供している．
* バージョン管理システムは，通常クライアント-サーバモデルであり，サーバ側にマスターデータを持ち，各開発者はそのサーバからソースを取得し，修正が完了したらコミットする．

バージョン管理システムを利用すると良いことばかりのようであるが，
以下のような短所もある．

* サーバで管理されているデータを取得するためにはバージョン管理システム専用のクライアントツールをインストールして利用する必要がある．
* 利用方法を習得する必要がある．

但し，上記の短所については，バージョンシステム自体が広く利用されているシステムであることから，大きな問題となることは通常ない．
以降に，バージョン管理システムとして良く利用されているSubersion，Git，Sourceforgeについて説明する．

Subversion
----------

概要
^^^^

Subversionとは，無償で利用できる集中型のバージョン管理システムの一つであり，Windows，Mac，Linuxなど多くのOS上で利用することができる．
Subversionはクライアント-サーバモデルというシステムの構成をとり，バージョン管理するデータはサーバ側のリポジトリと呼ばれるところでSubversionにより集中管理される．
クライアント側にはSubersion用の専用ツールをインストールし，サーバ側のリポジトリからデータを取得，修正後にコミットする．

Subversionは以下のような特徴を持つ．

* バージョン番号はファイル単位ではなく，ソースツリー全体に対して設定する．つまり，誰かがソースツリーのどこかのファイルを変更する度にバージョン番号が増える．
* 管理対象のファイル・ディレクトリの移動や削除を行うことができるため，開発するフォルダの構成が決まっていない開発初期段階からバージョン管理を行うことができる．
* クライアントとサーバの通信にsshをサポートしているため，インターネットを介したサーバとのデータのやりとりもセキュリティを保つことができる．

以下にSubversionを利用する場合のシステム構成について示す．

.. figure:: images/doc_subversion_structure.*

  Subversionのシステム構成

      .. csv-table:: Subversionクライアントソフトウェアの操作一覧
         :header: "操作名", "内容"
         :widths: 30, 80

         "checkout","リポジトリから作業コピーをチェックアウトします"
         "commit","作業コピーの変更点をリポジトリに送り，変更点を確定する"
         "update","作業コピーを更新します"
         "add","ファイル，ディレクトリ，シンボリックリンクをバージョン管理対象に追加します(commitで確定)"
         "delete","作業コピーやリポジトリから項目(ファイルなど)を削除します(commitで確定)"
         "copy","作業コピーやリポジトリ中の，ファイルやディレクトリをコピーします(commitで確定)"
         "move","ファイルやディレクトリを移動します(commitで確定)"
         "mkdir","バージョン管理下に新しいディレクトリを作成します(commitで確定)"
         "revert","ローカルファイルへのすべての編集を取り消します"
         "import","バージョン管理外のファイルやツリーを，リポジトリにコミットします"
         "diff","二つのパスやリビジョン間の差分を表示します"
         "status","作業コピーにあるファイルやディレクトリの状態を表示します"
         "info","ローカル，あるいはリモートパスにある項目についての情報を表示します"

導入
^^^^

Subversionはクライアント-サーバのシステム構成をとるため，本誌ではSubversionのクライアントソフトウェア，サーバソフトウェアを以下の構成で導入する．

.. figure:: images/doc_subversion_install_structure.*

  Subversion導入時のシステム構成

Subversionの導入手順(Windows編，Linux編)を以降に示す．

`Windows編`

WindowsにSubversionを導入する手順は以下の通り．

* Subversionサーバソフトウェアのインストール
* Subversionクライアントソフトウェアのインストール

1 *Subversionサーバソフトウェアのインストール*

\(1\) Subversionサーバソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://subversion.apache.org/packages.htmlの画面を開く．
* 画面下部にあるWindowsから環境に応じて以下のソフトウェアのいずれかのリンクをクリックする．
  (本誌ではVisualSVNを利用する．)

      .. csv-table:: Subversionサーバソフトウェアの一覧
         :header: "ソフトウェア", "内容"
         :widths: 100, 200

         "VisualSVN","VisualSVNによってサポート/メンテナンスされている．client and serverを含む．"
         "WANdisco","WANdiscoによってサポート/メンテナンスされている．32/64-bit client and serverを含む．"
         "Win32Svn","David Darjによってメンテナンスされている．32-bit client, server and bindings, MSI and ZIPs．"

* ダウンロード画面から「Apache Subversion command line tools」の右のDownloadリンクをクリックする．(2011年12月執筆時点の最新版Apache-Subversion-1.7.2.zipを取得)

\(2\) Subversionサーバソフトウェアのインストール

* ダウンロードした「Apache-Subversion-1.7.2.zip」を解凍する．
* 解凍したフォルダのbinをPATH環境変数に追加する．例) C:\Apache-Subversion-1.7.2\binをPATHに追加する．

2 *Subversionクライアントソフトウェアのインストール*

\(1\) Subversionクライアントソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://tortoisesvn.net/の画面を開く．
* 画面上部のDownloadsリンクをクリックし，表示された画面の「TortoiseSVN 32-Bit」のリンクをクリックする．

  .. warning::

     上記のインストーラは32bit版であり，64bit版を利用している場合は「TortoiseSVN 64-Bit」リンクをクリックすること．

* ダウンロードした「TortoiseSVN-1.7.3.22386-win32-svn-1.7.2.msi」ファイルをダブルクリックする．
* 支持に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_tortoiseclient_install-1.*

  Subversionクライアントソフトウェアのインストール(その1)

.. figure:: images/doc_tortoiseclient_install-2.*

  Subversionクライアントソフトウェアのインストール(その2)

.. figure:: images/doc_tortoiseclient_install-3.*

  Subversionクライアントソフトウェアのインストール(その3)

.. figure:: images/doc_tortoiseclient_install-4.*

  Subversionクライアントソフトウェアのインストール(その4)

.. figure:: images/doc_tortoiseclient_install-5.*

  Subversionクライアントソフトウェアのインストール(その5)

..


`Linux編`

 ※ Linux(Ubuntu)にはSubversionのクライアント/サーバソフトウェアがデフォルトでインストールされているため実施事項はない．

利用例
^^^^^^

Subversionの利用例(Windows編，Linux編)を以降に示す．

`Windows編`

\(1\) Subversion利用準備

* リポジトリディレクトリの作成
.. code-block:: commandprompt

   svnadmin  create C:\repository

- 匿名アクセスのアクセス権限の設定(匿名ユーザにコミット権限を与える場合)

 - 「C:\repository\conf\svnserve.conf」ファイルを開く
 - 19行目あたりの行を以下のように修正し，保存する．
.. code-block:: commandprompt

   修正前 : # anon-access = read
   修正後 : anon-access = write

- trunkディレクトリの作成
.. code-block:: commandprompt

   svn mkdir file:///C:\repository/trunk -m "mkdir trunk"

* リポジトリに「trunk」フォルダをimportする．(trunk/testDir/a.txtというデータを用意しておくこと)
.. code-block:: commandprompt

   svn import trunk file:///C:\repository/trunk/ -m "Initial import"

\(2\) Subversionサーバソフトウェアの準備

* Subversionサーバソフトウェアの起動
.. code-block:: commandprompt

   svnserve -d -r C:\repository\

   ※ Subversionサーバソフトウェアの停止は，svnserveプロセスの停止で行う．

\(3\) Subversionクライアントソフトウェアの利用例

* リポジトリから作業コピーディレクトリにcheckout

 * Cドライブ直下にsampleフォルダを作成する．(任意)
 * sampleフォルダ内に移動し，右クリック＞「SVN Checkout...」の選択する．
 * Checkout画面のURL of repository下のテキストフィールドに「svn://localhost/trunk」と入力し，OKボタンを押す．

.. figure:: images/doc_subversion_usecase-1.*

  Subversionチェックアウト画面

* ファイルの修正/コミット

 * testDir/a.txtを修正する．
 * testDirフォルダ上で右クリック＞「SVN Commit...」を選択する．
 * 表示された画面でOKボタンを押す．

* 新しいファイルの追加/コミット

 * testDirフォルダ内にb.txtを作成する．
 * b.txtを選択し，右クリック＞「TortoiseSVN＞「Add」を選択する．
 * testDirフォルダ上で右クリック＞「SVN Commit...」を選択する．
 * 表示された画面でOKボタンを押す．

.. figure:: images/doc_subversion_usecase-2.*

  Subversion登録後の画面

* 他の人が修正したファイルの取得

 * 他の人がa.txtファイルを修正し，コミットしていた場合，testDirフォルダ上で，右クリック＞「SVN Update...」を選択する．


`Linux編`

\(1\) Subversion利用準備

* リポジトリディレクトリの作成
.. code-block:: commandprompt

   svnadmin  create /var/svn_rep/repository

* 匿名アクセスのアクセス権限の設定(匿名ユーザにコミット権限を与える場合)

 * 「./repository/conf/svnserve.conf」ファイルを開く
 * 12行目あたりの行を以下のように修正し，保存する．
.. code-block:: bash

   修正前 : # anon-access = read
   修正後 : anon-access = write

* trunkディレクトリの作成
.. code-block:: commandprompt

   svn mkdir file:///var/svn_rep/repository/trunk -m "mkdir trunk"

* リポジトリに「trunk」ディレクトリをimportする．(trunk/testDir/a.txtというデータを用意しておくこと)
.. code-block:: commandprompt

   svn import trunk file:///var/svn_rep/repository/trunk/ -m "Initial import"

\(2\) Subversionサーバソフトウェアの準備

* Subversionサーバソフトウェアの起動
.. code-block:: commandprompt

   svnserve -d -r /var/svn_rep/repository

   ※ Subversionサーバソフトウェアの停止は，svnserveプロセスの停止で行う．

\(3\) Subversionクライアントソフトウェアの利用例

* リポジトリから作業コピーディレクトリにcheckout

 * ホーム直下にsampleディレクトリを作成する．(任意)
 * sampleディレクトリに移動し，データをチェックアウトする．
.. code-block:: bash

   svn checkout svn://localhost/trunk

* ファイルの修正/コミット

 * testDir/a.txtを修正する．
 * コミットする
.. code-block:: bash

   svn status
   M     testDir/a.txt  <-- Mは修正されていることを意味する

   svn commit -m "test commit"

* 新しいファイルの追加/コミット

 * testDirディレクトリ内にb.txtを作成する．
 * b.txtを追加し、コミットする．
.. code-block:: bash

   svn add testDir/b.txt
   A     testDir/b.txt
   svn commit testDir/b.txt -m "test commit"

* 他の人が修正したファイルの取得

 * 他の人がa.txtファイルを修正し，コミットしていた場合updateする．
.. code-block:: bash

   svn update

Git
---

概要
^^^^
Gitとは，無償で利用できる分散型のバージョン管理システムの一つであり，Linuxカーネルのソースコード管理を目的として，リーナス・トーバルズによって開発された．
Windows，Mac，Linuxなど多くのOS上で利用することができる．
Gitはクライアント-サーバモデルというシステム構成をとり，バージョン管理するデータはサーバ側の中央リポジトリ，クライアント側のローカルリポジトリと呼ばれるところでGitにより管理される．
クライアント側にはGit用の専用ツールをインストールし，リポジトリからデータを取得，修正後にコミットする．

なお，Subersionでは，データの変更は必ずサーバのリポジトリにコミットすることになるが，Gitの場合は，ローカルリポジトリにコミットし，その後，ローカルのリポジトリのデータをサーバ側の中央リポジトリに反映する．
Subersionでは，ソースコードを管理するためにはコミットする必要があるため，例えば，テストが実施できていないソースもバージョン管理するためにはコミットする必要があり，この操作が他の開発者に影響を与えることがあった．
Gitでは，ローカルリポジトリだけでバージョン管理することができるため，左記の問題を解消することができる．テストが完了した後に中央リポジトリに反映すればよい．

Gitは以下のような特徴を持つ．

* リポジトリがローカル，中央に分かれており，ローカルリポジトリだけでもバージョン管理ができる．
* 動作速度に重点が置かれたシステムである．
* リポジトリへのアクセスプロトコルには，ローカル，ssh，rsync，Git 独自プロトコル，WebDAVなどがある．

以下にGitを利用する場合のシステム構成について示す．

.. comment

   .. figure:: images/doc_git_structure.*

   Gitのシステム構成

..

      .. csv-table:: Gitクライアントソフトウェアの操作一覧
         :header: "操作名", "内容"
         :widths: 30, 80

         "add","ファイル，ディレクトリ，シンボリックリンクをバージョン管理対象に追加する"
         "clone","新しいディレクトリ内にリポジトリのクローンを作成する"
         "commit","作業コピーの変更点をリポジトリに送り，変更点を確定する"
         "diff","二つのパスやリビジョン間の差分を表示する"
         "init","空のリポジトリを作成する"
         "mv","ファイルやディレクトリを移動する(commitで確定)"
         "pull","MasterリポジトリからLocalリポジトリにデータを転送する"
         "push","LocalリポジトリからMasterリポジトリにデータを転送する"
         "rm","作業コピーやリポジトリから項目(ファイルなど)を削除する(commitで確定)"

導入
^^^^

Gitはクライアント-サーバのシステム構成をとるため，本誌ではGitのクライアントソフトウェア，サーバソフトウェアを以下の構成で導入する．

.. figure:: images/doc_git_install_structure.*

  Git導入時のシステム構成

Gitの導入手順(Windows編，Linux編)を以降に示す．

`Windows編`

WindowsにGitを導入する手順は以下の通り．

* Gitサーバソフトウェアのインストール
* Gitクライアントソフトウェアのインストール

1 *Gitサーバソフトウェアのインストール*

\(1\) Gitサーバソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，Gitのサーバソフトウェアであるmsysgitをダウンロードするhttp://code.google.com/p/msysgit/downloads/listの画面を開く．
* Git-1.7.8-preview20111206.exe(2011年12月執筆時点)のリンクをクリックする．

\(2\) Gitサーバソフトウェアのインストール

* ダウンロードした「Git-1.7.8-preview20111206.exe」を実行する．
* 支持に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_git_install-1.*

  Gitクライアントソフトウェアのインストール(その1)

.. figure:: images/doc_git_install-2.*

  Gitクライアントソフトウェアのインストール(その2)

.. figure:: images/doc_git_install-3.*

  Gitクライアントソフトウェアのインストール(その3)

.. figure:: images/doc_git_install-4.*

  Gitクライアントソフトウェアのインストール(その4)

.. figure:: images/doc_git_install-5.*

  Gitクライアントソフトウェアのインストール(その5)

.. figure:: images/doc_git_install-6.*

  Gitクライアントソフトウェアのインストール(その6)

.. figure:: images/doc_git_install-7.*

  Gitクライアントソフトウェアのインストール(その7)

..

2 *Gitクライアントソフトウェアのインストール*

 ※ msysgitはGitクライアントソフトウェアの機能も持つため，本作業は不要．

`Linux編`

1 *Gitのインストール*

\(1\) Gitのインストール
.. code-block:: bash

   sudo aptitude install git-core

.. comment

   p1414@p1414-desktop:~$ sudo aptitude install git-core
   [sudo] password for p1414:
   Sorry, try again.
   [sudo] password for p1414:
   パッケージリストを読み込んでいます... 完了
   依存関係ツリーを作成しています
   状態情報を読み取っています... 完了
   拡張状態情報を読み込んでいます
   パッケージの状態を初期化しています... 完了
   拡張状態情報を書き込んでいます... 完了
   以下の新規パッケージがインストールされます:
     git-core libdigest-sha1-perl{a} liberror-perl{a} patch{a}
   0 個のパッケージを更新、 4 個を新たにインストール、 0 個を削除予定、206 個が更新されていない。
   6,315kB のアーカイブを取得する必要があります。 展開後に 13.1MB のディスク領域が新たに消費されます。
   先に進みますか? [Y/n/?] Y
   拡張状態情報を書き込んでいます... 完了
   取得:1 http://jp.archive.ubuntu.com/ubuntu/ lucid/main liberror-perl 0.17-1 [23.8kB]
   取得:2 http://jp.archive.ubuntu.com/ubuntu/ lucid/main libdigest-sha1-perl 2.12-1build1 [26.7kB]
   取得:3 http://jp.archive.ubuntu.com/ubuntu/ lucid-updates/main git-core 1:1.7.0.4-1ubuntu0.2 [6,143kB]
   取得:4 http://jp.archive.ubuntu.com/ubuntu/ lucid/main patch 2.6-2ubuntu1 [121kB]
   6,315kB を 15s 秒でダウンロードしました (406kB/s)
   未選択パッケージ liberror-perl を選択しています。
   (データベースを読み込んでいます ... 現在 156889 個のファイルとディレクトリがインストールされています。)
   (.../liberror-perl_0.17-1_all.deb から) liberror-perl を展開しています...
   未選択パッケージ libdigest-sha1-perl を選択しています。
   (.../libdigest-sha1-perl_2.12-1build1_amd64.deb から) libdigest-sha1-perl を展開しています...
   未選択パッケージ git-core を選択しています。
   (.../git-core_1%3a1.7.0.4-1ubuntu0.2_amd64.deb から) git-core を展開しています...
   未選択パッケージ patch を選択しています。
   (.../patch_2.6-2ubuntu1_amd64.deb から) patch を展開しています...
   man-db のトリガを処理しています ...
   liberror-perl (0.17-1) を設定しています ...
   libdigest-sha1-perl (2.12-1build1) を設定しています ...
   git-core (1:1.7.0.4-1ubuntu0.2) を設定しています ...
   patch (2.6-2ubuntu1) を設定しています ...
   パッケージリストを読み込んでいます... 完了
   依存関係ツリーを作成しています
   状態情報を読み取っています... 完了
   拡張状態情報を読み込んでいます
   パッケージの状態を初期化しています... 完了
   拡張状態情報を書き込んでいます... 完了

   p1414@p1414-desktop:~$

..

利用例
^^^^^^

Gitの利用例(Windows編，Linux編)を以降に示す．

`Windows編`

\(1\) Git利用例

* Gitユーザの登録
 * Git Bashの実行
 * Gitユーザの登録
.. code-block:: bash

   git config --global user.name "testUser"

* Local1リポジトリディレクトリの作成
 * Git Bashを実行し、Git Bash画面から「C:\gitrep\」フォルダ，リポジトリの作成を行う．
.. code-block:: bash

   mkdir /c/glocal1
   cd /c/glocal1
   git init

* Masterリポジトリディレクトリの作成
 * Git Bashを実行し、Git Bash画面から「C:\gitrep\」フォルダ，リポジトリの作成を行う．
.. code-block:: bash

   mkdir /c/gmaster
   cd /c/gmaster
   git --bare init

* Local1リポジトリにファイルのコミット
 * Git Bashを起動し，以下を実行する．
.. code-block:: bash

   cd /c/glocal1
   touch first
   git add first
   git commit first -m "first commit"

* Local1リポジトリ(master)をMasterリポジトリ(origin)にプッシュ
 * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   cd /c/glocal1
   git remote add origin /c/gmaster
   git push origin master

* Masterリポジトリ(origin)の作業コピーディレクトリ(Local2リポジトリ)を作成する．
 * Git Bashを起動し，以下を実行する．
.. code-block:: bash

   mkdir /c/glocal2
   cd /c/glocal2
   git clone /c/gmaster

* Local2リポジトリに追加したファイルをLocal1リポジトリに反映
 * Git Bashを起動し，以下を実行する．
.. code-block:: bash

   cd /c/glocal2/gmaster
   touch second
   git add second
   git commit second -m "second commit"
   git push origin master 

   cd /c/glocal1
   git pull origin master

..

`Linux編`

\(1\) Git利用例

* Gitユーザの登録
.. code-block:: bash

   git config --global user.name "testUser"

* Local1リポジトリディレクトリの作成
 * /home/p1414/glocal1というリポジトリの作成を行う．
.. code-block:: bash

   mkdir /home/p1414/glocal1
   cd /home/p1414/glocal1
   git init

* Masterリポジトリディレクトリの作成
 * /home/p1414/gmasterというリポジトリの作成を行う．
.. code-block:: bash

   mkdir /home/p1414/gmaster
   cd /home/p1414/gmaster
   git --bare init

* Local1リポジトリにファイルのコミット
.. code-block:: bash

   cd /home/p1414/glocal1
   touch first
   git add first
   git commit first -m "first commit"

* Local1リポジトリ(master)をMasterリポジトリ(origin)にプッシュ
.. code-block:: bash

   cd /home/p1414/glocal1
   git remote add origin /home/p1414/gmaster
   git push origin master

* Masterリポジトリ(origin)の作業コピーディレクトリ(Local2リポジトリ)を作成する．
 * Git Bashを起動し，以下を実行する．
.. code-block:: bash

   mkdir /home/p1414/glocal2
   cd /home/p1414/glocal2
   git clone /home/p1414/gmaster

* Local2リポジトリに追加したファイルをLocal1リポジトリに反映
.. code-block:: bash

   cd /home/p1414/glocal2/gmaster
   touch second
   git add second
   git commit second -m "second commit"
   git push origin master 

   cd /home/p1414/glocal1
   git pull origin master

Sourceforge.JP
--------------

概要
^^^^

SourceForge.JP（ソースフォージドットジェーピー）は，日本のオープンソースソフトウェアプロジェクト向けのホスティングサイトである．
SourceForge.JPは以下のようなサービスを提供している．

* CVS/SVN/Git/Mercurial/Bazaarリポジトリ．ソースコードのバージョン管理が行える．
* プロジェクトWiki．プロジェクト開発ドキュメントを管理することができる．wikiの記法はSourceForge.JP独自のものである．
* プロジェクトWeb．ホスティングされているプロジェクトが自由に使えるWebスペースで，CGI等も自由に設置できる．なお，その際のサイト名はプロジェクト名.sourceforge.jpもしくは，独自ドメイン．
* シェルサーバ．シェルの機能を利用するためのサーバを利用することができる．
* トラッカー．バグ報告，機能の追加要望等を管理できるツール．
* ML/フォーラム．メーリングリストとディスカッションフォーラムを利用することができる．
* ファイルリリース/ダウンロードミラー．ソフトウェアのパッケージを配布するためのツールを利用することができる．

SourceForge.JPのサービスを利用するにあたりホスティング費用は発生しないが，オープンソースプロジェクトホスティングサイトであるため，開発成果はオープンソースとして公開する必要がある．
なお，ライセンスはOpen Source Initiativeにオープンソースライセンスとして承認されているもの(GPL，LGPL，Apache License 2.0など)が利用可能である．



.. todo:: コーディング規約についての記述が必要


テスティングフレームワーク
==========================

テストの必要性
--------------

ソフトウェアの開発は通常以下の手順で行われる．

* ソフトウェアの設計
* ソフトウェアの作成
* ソフトウェアのテスト
* ソフトウェアの利用

ソフトウェアを利用していると，不具合，使い勝手が悪い，追加の機能が欲しいなどの理由から
再度上記の手順を繰り返す場合が多い．その手順を繰り返していくと徐々にソフトウェアの
規模が大きくなり，修正したプログラムが他のプログラムに与える影響を把握することが難しくなっていく．
結果として，修正したプログラムの問題により，他のプログラムが正常に動かなくなることもある．

よって，ソフトウェアの変更を行う際には，ソフトウェアの品質を維持するために変更した部分のみならず，変更していない部分のテストも行う必要がある．

ここで，継続的インテグレーション（Continuous Integration，CIと略すこともある）と呼ぶソフトウェア開発手法について説明する．
継続的インテグレーションとは，ビルド(コンパイル)・テストなどを自動化し，これらの作業を1日に何度も 繰り返すことで，ビルド，テストの失敗を早期に発見し，ソフトウェアの品質維持，納期短縮を行うためのソフトウェア・エンジニアリングの習慣の集合である．

この開発手法によるソフトウェア開発を行うことで以下のようなメリットを受けることができる．

* 自動でビルド，テストが行われているため，開発したソフトウェアに対する品質を継続的に維持できる．(品質維持)
* 追加・変更したソフトウェアに問題があった場合，早期にその問題を発見できる．早期に発見できると問題の原因特定が比較的容易に行えることが多く，結果として開発期間を短くすることができる．(納期短縮)
以降では，Jenkinsと呼ぶ，継続的インテグレーションを実践するためのソフトウェアについて説明する．


Jenkins
-------

概要
^^^^

Jenkinsとは，継続的インテグレーションのためのソフトウェアである．
Jenkinsはソフトウェアのビルド，テストを継続的に行うための仕組みを持ち，
エラーが発生した場合，ユーザに通知したりすることができる．
 
Jenkinsの特徴を以下に示す．

* インストールが容易．
* 設定が容易．XMLなどのファイルを修正する必要はない．
* RSS/メールでビルド結果(成功，失敗)を通知することができる．
* Subversion，Git，Mercurial，Bazaarなどのソースコード管理ツールと連携し，自動で最新のテスト対象のソースを取得することができる．


導入
^^^^

JenkinsをLinuxにインストール・起動する手順を以降に示す．

1 *Jenkinsのインストール*

  .. code-block:: bash
    :linenos:
 
    wget -q -O - http://pkg.jenkins-ci.org/debian/jenkins-ci.org.key | 
      sudo apt-key add -
    sudo sh -c 'echo deb http://pkg.jenkins-ci.org/debian binary/ > 
      /etc/apt/sources.list.d/jenkins.list'
    sudo aptitude update
    sudo aptitude install jenkins

    ※ 更新する場合
       sudo aptitude update
       sudo aptitude install jenkins

2 *Jenkinsの起動*

\(1\) 起動

  .. code-block:: bash
    :linenos:

    java -jar jenkins.war

    ※ デフォルトの環境でjenkinsをインストールした場合，
       jenkinsは/usr/share/jenkinsにインストールされている．

\(2\) 設定


.. comment

   Jenkins home directory: /home/p1414/.jenkins found at: $user.home/.jenkins
   [Winstone 2011/12/03 19:03:16] - HTTP Listener started: port=8080
   [Winstone 2011/12/03 19:03:16] - AJP13 Listener started: port=8009
   [Winstone 2011/12/03 19:03:16] - Winstone Servlet Engine v0.9.10 running: controlPort=disabled
   2011/12/03 19:03:17 jenkins.model.Jenkins$6 onAttained
   情報: Started initialization
   2011/12/03 19:03:17 jenkins.model.Jenkins$6 onAttained
   情報: Listed all plugins
   2011/12/03 19:03:18 jenkins.model.Jenkins$6 onAttained
   情報: Prepared all plugins
   2011/12/03 19:03:18 jenkins.model.Jenkins$6 onAttained
   情報: Started all plugins
   2011/12/03 19:03:18 jenkins.model.Jenkins$6 onAttained
   情報: Augmented all extensions
   2011/12/03 19:03:18 jenkins.model.Jenkins$6 onAttained
   情報: Loaded all jobs
   2011/12/03 19:03:21 jenkins.model.Jenkins$6 onAttained
   情報: Completed initialization
   2011/12/03 19:03:21 hudson.TcpSlaveAgentListener <init>
   情報: JNLP slave agent listener started on TCP port 47984
   2011/12/03 19:03:32 hudson.WebAppMain$2 run
   情報: Jenkins is fully up and running


.. todo:: 起動するプロジェクトの作成が必要


