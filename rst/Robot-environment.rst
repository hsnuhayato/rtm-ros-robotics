ロボット開発環境
================


ドキュメンテーションフレームワーク
----------------------------------

ドキュメントの必要性
^^^^^^^^^^^^^^^^^^^^

ソフトウェア開発におけるドキュメントの必要性について示す．
まず，ソフトウェア開発には以下のような特徴がある．

* ソフトウェア開発は，「設計」→「実装」→「試験」の手順で行う．
* ソフトウェア開発は，作成するソフトウェアの規模，期間に応じて，1人から複数の人で行う．
* 一度作成したソフトウェアに対する不具合の対応，機能追加を行うことがある．機能追加前後の開発者が異なることもある．

複数の人で開発を行う場合は，ソフトウェアの設計情報を共有する必要があり，また，一度作成したソフトウェアに対する機能追加を行うような場合，どのような構成でソフトウェアが作成されているかを確認する必要がある．
よって，ソフトウェア開発を行う際には，ソフトウェアの構成等を示すドキュメントを作成しておく必要がある．

..

ここで，ドキュメントの作成には，従来Microsoft OfficeのWordなどの専用のソフトウェアを利用することが多かったと思われる．専用のソフトウェアの場合，ソフトウェアがインストールされた計算機上でしかドキュメントを作成することができない，また，GUIの作業環境が必要であるなどの問題点があった．
これらの問題を解決する方法の一つとして，現在注目を浴びているreStructruedTextというマークアップ言語，Sphinxというソフトウェアを用いたドキュメント作成方法を以降の章で紹介する．

reStructuredText / Sphinx
^^^^^^^^^^^^^^^^^^^^^^^^^

*概要*

reStructuredTextとは，文書の構造や表示の仕方などを定義したマークアップ言語のひとつである．
reStructuredText は RST，ReST，reSTと略されることもある．
reStructuredTextの特徴は以下の通り．

* テキスト形式で記述するため，計算機に標準でインストールされているメモ帳などのテキストエディタで作成することができる．(専用のソフトウェアのインストールは不要．つまり，文章が作成できるあらゆる計算機でドキュメントを作成できる．)
* ソースコードの状態でも高い可読性を持っている．
* reStructuredTextで記述された文書はPDF，HTML，XML，LaTeXなど複数の形式の文書に変換することができる．

Sphinxとはドキュメントを簡単に作成できるようにするためのソフトウェアである．
SphinxはドキュメントをreStructuredTextで記述し，作成したドキュメントをHTML，PDFなどの形式に変換することができる．
Sphinxの特徴は以下の通り．

* 実行環境はWindows/Linux/Mac OS，Python 2.4以上のインストールが必要である．
* 出力フォーマットはPDF，HTML，XML，LaTeX，ePubなどをサポートしている．
* 引用，用語解説，ソースコードへのクロスリファレンス機能．
* 引用したソースコードを自動でハイライトする機能．
* 章などのインデックスを自動で設定する機能．

*導入*

以降に，Sphinxを導入するための手順を示す．なお，SphinxはPythonで作成されていることからPythonが実行できる環境であればどこでも動作する．
よって，本誌ではWindows，Linuxそれぞれの環境にPythonのeasy_install(パッケージ管理システムからPythonのモジュールを自動で検索してインストールやアップデートしてくれるツール)を利用したSphinxの導入手順を示す．

`Windows`

WindowsにSphinxを導入する手順は以下の通り．

* Pythonのインストール
* easy_installのインストール
* Sphinxのインストール

1 *Pythonのインストール*

Pythonのインストール手順を示す．なお，既にインストールされている場合は，本手順は不要である．

\(1\) Pythonのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://python.org の画面を開く．

.. figure:: images/doc_python_install-1.*

  Pythonのインストール画面(1/6)

* 画面左端の「Download」リンクをクリックする．
* 画面上部の「Python 2.7.2 Windows Installer」リンク(2011年12月 執筆時点の最新版)をクリックする．

.. figure:: images/doc_python_install-2.*

  Pythonのインストール画面(2/6)

* ダウンロード画面が表示されるため，「OK」ボタンを押す．

  .. warning::

     上記のインストーラは32bit版であり，64bit版を利用している場合は「Python 2.7.2 Windows X86-64 Installer 」リンクをクリックすること．

\(2\) Pythonのインストール

* ダウンロードした「python-2.7.2.msi」ファイルをダブルクリックする．
* 指示に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_python_install-3.*

  Pythonのインストール画面(3/6)

.. figure:: images/doc_python_install-4.*

  Pythonのインストール画面(4/6)

.. figure:: images/doc_python_install-5.*

  Pythonのインストール画面(5/6)

.. figure:: images/doc_python_install-6.*

  Pythonのインストール画面(6/6)

..

2 *easy_installのインストール*

easy_installとは，パッケージ管理システムからPythonのモジュールを自動で検索し，インストールやアップデートをするツールである．
Sphinxはこのツールを利用してインストールする．
以降に，easy_installのインストール手順を示す．なお，既にインストールされている場合は，本手順は不要である．

\(1\) easy_installファイルのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://peak.telecommunity.com/dist/ez_setup.py の画面を開く．

.. figure:: images/doc_easy_install_install-1.*

  easy_installのインストール画面(1/2)

* 表示された画面上で右クリックをし，「名前を付けてページを保存」を実行する．なお，その際に保存するファイル名は「ez_setup.py」とし，Cドライブ直下に保存する．

\(2\) easy_installのインストール

* コマンドプロンプト画面を開く．（コマンドプロンプト画面は，スタート->プログラム->アクセサリ->コマンドプロンプト の手順で表示することができる）
* コマンドプロンプト画面からCドライブ直下に移動する．(コマンドプロンプト画面で「cd C:\\」を入力後，Enterを押すことでCドライブ直下に移動できる)
* コマンドプロンプト画面で「python ez_setup.py」を入力後，Enterを押す．

..

   .. figure:: images/doc_easy_install_install-2.*
  
     easy_installのインストール画面(2/2)

..

3 *Sphinxのインストール*

\(1\) Sphinxのインストール

* コマンドプロンプト画面を開く．
* コマンドプロンプト画面で「easy_install sphinx」を入力後，Enterを押す．

.. figure:: images/doc_sphinx_install.*

  Sphinxのインストール画面

..

`Linux`

Linux(Ubuntu)にSphinxをインストールする方法は，「パッケージシステムを利用したインストール」と「手動インストール」の2つがある．
「手動インストール」については，Sphinxのインストール(Windows)の「2 easy_installのインストール」，「3 Sphinxのインストール」と同様である．
以降には，「パッケージシステムを利用したインストール」手順を示す．

1 *パッケージシステムを利用したインストール*

ターミナル画面から以下のコマンドを実行する．

  .. code-block:: bash

    aptitude install python-sphinx

.. 以下コメントアウト
 
   上記コマンドの実行結果の内容は以下の通り．

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

*ドキュメント作成*

Sphinxを利用したドキュメント作成方法をWindows，Linuxそれぞれの環境に分けて以降に示す．なお，作成するドキュメントの構成は，目次，章が2つのものである．

`Windows`

1 *プロジェクトの作成*

Sphinxではプロジェクトという単位でドキュメントを作成する．
プロジェクト情報は以下とする．

      .. csv-table:: Sphinxのプロジェクト情報(Windows)
         :header: "項目", "内容"
         :widths: 20, 20

         "プロジェクトの作成場所","C:\\sample-project"
         "プロジェクト名","sample-project"
         "バージョン番号","2012.01.01"

..
..

\(1\) sphinx-quickstartの実行

sphinx-quickstartとは，Sphinxのプロジェクトを作成するコマンドである．実行手順を以下に示す．

* コマンドプロンプト画面を開く．
* コマンドプロンプト画面で「mkdir C:\\sample-project」を入力後，Enterを押し，プロジェクトフォルダを作成する．
* コマンドプロンプト画面でC:\\sample-project直下に移動する．(コマンドプロンプト画面で「cd C:\\sample-project」を入力後，Enterを押すことで移動できる)
* コマンドプロンプト画面で「sphinx-quickstart」を入力後，Enterを押し，プロジェクト情報を入力する．なお，以降の★で示す，「プロジェクト名」，「バージョン番号」，「著者の名前」以外はデフォルトでも特に問題ない．設定内容の詳細は 「Sphinxの日本ユーザ会」のページを参照．http://sphinx-users.jp/gettingstarted/sphinxquickstart.html．

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

2 *ドキュメントの作成*

sphinx-quickstartで作成したプロジェクト内にドキュメントを作成する．
なお，ドキュメント構成は以下とする．

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
* 以後，rstファイルを修正していけば良い．

.. figure:: images/doc_sphinx_win-html-1.*

  Sphinxで作成した画面

`Linux`

1 *プロジェクト作成*

Sphinxではプロジェクトという単位でドキュメントを作成する．
プロジェクト情報は以下とする．

      .. csv-table:: Sphinxのプロジェクト情報(Linux)
         :header: "項目", "内容"
         :widths: 20, 20

         "プロジェクトの作成場所","/home/testUser/sample-project"
         "プロジェクト名","sample-project"
         "バージョン番号","2012.01.01"
         "著者の名前","sample"

..
..

\(1\) sphinx-quickstartの実行

sphinx-quickstartとは，Sphinxのプロジェクトを作成するコマンドである．実行手順を以下に示す．

* ターミナル画面を開く．
* ターミナル画面でtestUserユーザのホームディレクトリ(/home/testUser)に移動し，ホームディレクトリ直下にsample-projectディレクトリを作成する．(mkdir sample-project)
* ターミナル画面で「sphinx-quickstart」を入力後，Enterを押し，プロジェクト情報を入力する．なお，以降の★で示す，「プロジェクト名」，「バージョン番号」，「著者の名前」以外はデフォルトでも特に問題ない．設定内容の詳細は 「Sphinxの日本ユーザ会」のページを参照．http://sphinx-users.jp/gettingstarted/sphinxquickstart.html．

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

2 *ドキュメントの作成*

sphinx-quickstartで作成したプロジェクト内にドキュメントを作成する．
なお，ドキュメント構成は以下とする．

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

.. figure:: images/doc_sphinx_linux-html-1.*

  Sphinxで作成した画面

ソースコードリポジトリ
----------------------

ソースコードのバージョン管理
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ソフトウェアの開発では日常的にファイルの追加，修正を行うため，定期的にバックアップを取ることが重要である．バックアップをとる場合，通常ファイル名やフォルダ名に日付などを追加するが，この方法には以下のような問題がある．

* 前回のバックアップからの変更点がわからない．(変更履歴の問題)
* 毎回全てのデータを保存することになるため，ディスク容量を必要以上に使用してしまう．(ディスク容量の問題)

上記の問題を解決するためのシステムをバージョン管理システムと呼び，現在のソフトウェア開発では一般的に利用されている．ここで，バージョン管理システムには以下のような特徴がある．

* ファイルの変更履歴を管理し，変更履歴から変更点の比較が行える．また，過去のファイルを取り出すこともできる．誤って削除してしまっても元に戻すことができる．
* ファイルの変更点の管理は，通常前回データの差分のみであり，ディスク容量を必要以上に使用しない．
* 多くのバージョン管理システムは複数の人の利用を想定しており，複数の人が同時に同一のファイルを修正した場合の問題を解決する仕組みを提供している．
* バージョン管理システムは，通常クライアント-サーバモデルであり，サーバ側にマスターデータを持ち，各開発者はそのサーバからソースを取得し，修正が完了したらコミットする．

バージョン管理システムを利用すると良いことばかりのようであるが，
以下のような短所もある．

* サーバで管理されているデータを取得するためにはバージョン管理システム専用のクライアントツールをインストールして利用する必要がある．
* 利用方法を習得する必要がある．

但し，上記の短所については，バージョンシステム自体が広く利用されているシステムであることから，大きな問題となることは通常ない．
以降に，バージョン管理システムとして良く利用されているSubersion，Git，Sourceforgeについて説明する．

Subversion
^^^^^^^^^^

*概要*

Subversionとは，無償で利用できる集中型のバージョン管理システムの一つであり，Windows，Mac，Linuxなど多くのOS上で利用することができる．
Subversionはクライアント-サーバモデルというシステムの構成をとり，バージョン管理するデータはサーバ側のリポジトリと呼ばれるところでSubversionにより集中管理される．
クライアント側にはSubersion用の専用ツールをインストールし，サーバ側のリポジトリからデータを取得，修正後にコミットする．

Subversionは以下のような特徴を持つ．

* バージョン番号はファイル単位ではなく，ソースツリー全体に対して設定する．つまり，誰かがソースツリーのどこかのファイルを変更する度にバージョン番号が増える．
* 管理対象のファイル・ディレクトリの移動や削除を行うことができるため，開発するフォルダの構成が決まっていない開発初期段階からバージョン管理を行うことができる．
* クライアントとサーバの通信にsshをサポートしているため，インターネットを介したサーバとのデータのやりとりもセキュリティを保つことができる．
* リポジトリへのアクセスプロトコルには，ローカル，Subversion 独自プロトコル(sshあり、なし)，http，https．

以下にSubversionを利用する場合のシステム構成について示す．

.. figure:: images/doc_subversion_structure.*

  Subversionのシステム構成

以降でSubversionを利用する前に，Subversionについて最低限理解しておくべき概念，用語を以下に示す．

..

 *リポジトリ*

  Subversion で管理されるファイルの格納場所．変更履歴をリビジョンという番号を付与して管理している．

 *作業コピー*

  作業を行うために，リポジトリから取得したファイルを示す．Subversionはリポジトリのファイルを直接変更することはできないため，一旦リポジトリから作業コピーを作成し，これらのファイルに対して変更を加え，変更内容をリポジトリに反映させる．

 *checkout*

  リポジトリで管理されるファイルをSubversionクライアント計算機に全て取得する操作．

 *update*

  作業コピーとリポジトリの差分を比較し，作業コピーにある最新版以外のファイルをリポジトリから取得する操作．最初にckeckoutし，その後は，updateして作業コピーを最新版に保つ．

 *commit*

  作業コピーに対する変更操作をリポジトリに反映する操作．commitした時にcommitしたファイル群に新しいリビジョンが設定される．
 
 *import*

  Subversion管理対象外（リポジトリで管理されていない）のファイルをリポジトリにcommitする操作．
 
 *add*

  Subversion管理対象外（リポジトリで管理されていない）のファイルを管理対象とする操作．



*導入*

以降に，Subversionを導入するための手順を示す．なお，Subversionのインストールには様々な方法があるが，本誌ではSubversionクライアント-サーバ計算機の構成が共にWindows，Linuxの場合について示す．

`Windows`

以下の図に示す構成でSubversionの導入を行う．

.. figure:: images/doc_subversion_structure-install-win.*

  WindowsにSubversionを導入する場合のシステム構成

1 *Subversionサーバソフトウェアのインストール*

本作業はwsv計算機上で行うこと．

\(1\) Subversionサーバソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://subversion.apache.org/packages.htmlの画面を開く．
* 画面下部にあるWindowsから環境に応じて以下のソフトウェアのいずれかのリンクをクリックする．
  (本誌ではVisualSVNを利用する．)

      .. csv-table:: Subversionサーバソフトウェアの一覧
         :header: "ソフトウェア", "内容"
         :widths: 150, 200

         "VisualSVN","VisualSVNによってサポート/メンテナンスされている．client and serverを含む．"
         "WANdisco","WANdiscoによってサポート/メンテナンスされている．32/64-bit client and serverを含む．"
         "Win32Svn","David Darjによってメンテナンスされている．32-bit client, server and bindings, MSI and ZIPs．"

* ダウンロード画面から「Apache Subversion command line tools」の右のDownloadリンクをクリックする．(2011年12月執筆時点の最新版Apache-Subversion-1.7.2.zipを取得)

.. figure:: images/doc_subversion_install-1.*

  Subversionのインストール(1/1)

\(2\) Subversionサーバソフトウェアのインストール

* ダウンロードした「Apache-Subversion-1.7.2.zip」を解凍する．
* 解凍したフォルダのbinをPATH環境変数に追加する．例) C:\Apache-Subversion-1.7.2\binをPATHに追加する．

2 *Subversionクライアントソフトウェアのインストール*

本作業はwcl計算機上で行うこと．

\(1\) Subversionクライアントソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，http://tortoisesvn.net/の画面を開く．
* 画面上部のDownloadsリンクをクリックし，表示された画面の「TortoiseSVN 32-Bit」のリンクをクリックする．

.. figure:: images/doc_tortoiseclient_install-1.*

  Subversionクライアントソフトウェアのインストール(1/6)

..

  .. warning::

     上記のインストーラは32bit版であり，64bit版を利用している場合は「TortoiseSVN 64-Bit」リンクをクリックすること．

* ダウンロードした「TortoiseSVN-1.7.3.22386-win32-svn-1.7.2.msi」ファイルをダブルクリックする．
* 支持に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_tortoiseclient_install-2.*

  Subversionクライアントソフトウェアのインストール(2/6)

.. figure:: images/doc_tortoiseclient_install-3.*

  Subversionクライアントソフトウェアのインストール(3/6)

.. figure:: images/doc_tortoiseclient_install-4.*

  Subversionクライアントソフトウェアのインストール(4/6)

.. figure:: images/doc_tortoiseclient_install-5.*

  Subversionクライアントソフトウェアのインストール(5/6)

.. figure:: images/doc_tortoiseclient_install-6.*

  Subversionクライアントソフトウェアのインストール(6/6)

..

`Linux`

以下の図に示す構成でSubversionの導入を行う．

.. figure:: images/doc_subversion_structure-install-linux.*

  LinuxにSubversionを導入する場合のシステム構成

..

 ※ Ubuntu 10.04にはSubversionのクライアント/サーバソフトウェアであるsvnがデフォルトでインストールされているため実施事項はない．

*利用例*

Subversionの利用例をWindows，Linuxそれぞれの環境に分けて以降に示す．

`Windows`

\(1\) Subversionサーバソフトウェアの利用準備

本作業はwsv計算機上で行うこと．

* リポジトリの作成
.. code-block:: commandprompt

   svnadmin  create C:\\repository

* 匿名アクセスのアクセス権限の設定(匿名ユーザにコミット権限を与える場合)

  * 「C:\repository\conf\svnserve.conf」ファイルを開く
  * 19行目あたりの行を以下のように修正し，保存する．
.. code-block:: commandprompt

   修正前 : # anon-access = read
   修正後 : anon-access = write

* trunkディレクトリの作成
.. code-block:: commandprompt

   svn mkdir file:///C:\repository/trunk -m "mkdir trunk"

* リポジトリに「trunk」フォルダをimportする．(trunk/testDir/a.txtというデータを用意しておくこと)
.. code-block:: commandprompt

   svn import trunk file:///C:\repository/trunk/ -m "Initial import"

* Subversionサーバソフトウェアの起動
.. code-block:: commandprompt

   svnserve -d -r C:\repository\

   ※ Subversionサーバソフトウェアの停止は，svnserveプロセスの停止で行う．

\(2\) Subversionクライアントソフトウェアの利用例

本作業はwcl計算機上で行うこと．

* リポジトリから作業コピーディレクトリにcheckout

  * Cドライブ直下にsampleフォルダを作成する．(任意)
  * sampleフォルダ内に移動し，右クリック＞「SVN Checkout...」の選択する．
  * Checkout画面のURL of repository下のテキストフィールドに「svn://wsv/trunk」と入力し，OKボタンを押す．

.. figure:: images/doc_subversion_usecase-1.*

  Subversionチェックアウト画面

* ファイルの修正/コミット

  * testDir/a.txtを修正する．
  * testDirフォルダ上で右クリック＞「SVN Commit...」を選択する．

.. figure:: images/doc_subversion_commit.*

  Subversionのコミット画面

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


`Linux`

\(1\) Subversionサーバソフトウェアの利用準備

本作業はwsv計算機上で行うこと．

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

* Subversionサーバソフトウェアの起動
.. code-block:: commandprompt

   svnserve -d -r /var/svn_rep/repository

   ※ Subversionサーバソフトウェアの停止は，svnserveプロセスの停止で行う．

\(2\) Subversionクライアントソフトウェアの利用例

本作業はwcl計算機上で行うこと．

* リポジトリから作業コピーディレクトリにcheckout

  * ホーム直下にsampleディレクトリを作成する．(任意)
  * sampleディレクトリに移動し，データをチェックアウトする．
.. code-block:: bash

   svn checkout svn://wsv/trunk

* ファイルの修正/コミット

  * testDir/a.txtを修正する．
  * コミットする
.. code-block:: bash

   svn status
   M     testDir/a.txt  <-- Mは修正されていることを意味する

   svn commit -m "test commit"

* 新しいファイルの追加/コミット

  * testDirディレクトリ内にb.txtを作成する．
  * b.txtを追加し，コミットする．
.. code-block:: bash

   svn add testDir/b.txt
   A     testDir/b.txt
   svn commit testDir/b.txt -m "test commit"

* 他の人が修正したファイルの取得

  * 他の人がa.txtファイルを修正し，コミットしていた場合updateする．
.. code-block:: bash

   svn update

Git
^^^

*概要*

Gitとは，無償で利用できる分散型のバージョン管理システムの一つであり，Linuxカーネルのソースコード管理を目的として，リーナス・トーバルズによって開発された．
Windows，Mac，Linuxなど多くのOS上で利用することができる．
Gitはクライアント-サーバモデルというシステム構成をとり，バージョン管理するデータはサーバ側の中央リポジトリ，クライアント側のローカルリポジトリと呼ばれるところでGitにより管理される．
クライアント側にはGit用の専用ツールをインストールし，リポジトリからデータを取得，修正後にコミットする．

なお，Subersionでは，データの変更は必ずサーバのリポジトリにコミットすることになるが，Gitの場合は，ローカルリポジトリにコミットし，その後，ローカルのリポジトリのデータをサーバ側の中央リポジトリに反映する．
Subersionでは，ソースコードを管理するためにはコミットする必要があるため，例えば，テストが実施できていないソースもバージョン管理するためにはコミットする必要があり，この操作が他の開発者に影響を与えることがあった．
Gitでは，ローカルリポジトリだけでバージョン管理することができるため，テストが完了した後に中央リポジトリに反映するなどの対応で，上記の問題を解消することができる

Gitは以下のような特徴を持つ．

* リポジトリがローカル，中央に分かれており，ローカルリポジトリだけでもバージョン管理ができる．
* 動作速度に重点が置かれたシステムである．
* リポジトリへのアクセスプロトコルには，ローカル，ssh，rsync，Git 独自プロトコル，WebDAVなどがある．

以下にGitを利用する場合のシステム構成について示す．


.. figure:: images/doc_git_structure.*

   Gitのシステム構成

..

以降でGitを利用する前に，Gitについて最低限理解しておくべき概念，用語を以下に示す．

 *中央リポジトリ*

  Gitで管理されるファイルの格納場所．変更履歴をリビジョンという番号を付与して管理している．

 *ローカルリポジトリ*

  中央リポジトリのcloneとして作成したGitで管理されるファイルの格納場所．中央リポジトリ，ローカルリポジトリと呼び方は変えているが，管理上の呼び方を変えているだけである．どちらを中央と考えるかは利用者次第である．ローカルリポジトリ内でも変更履歴をリビジョンという番号を付与して管理している．

 *init*

  空のリポジトリを作成する．

 *clone*

  新しいディレクトリ内にリポジトリのクローンを作成する．

 *push*

  ローカルリポジトリから中央リポジトリにデータを転送する．

 *pull*

  中央リポジトリからローカルリポジトリにデータを転送する．

 *commit*

  作業コピーの変更点をローカルリポジトリに送り，変更点を確定する

 *add*

  Git管理対象外（リポジトリで管理されていない）のファイルを管理対象とする操作．


*導入*

以降に，Gitを導入するための手順を示す．なお，Gitのインストールには様々な方法があるが，本誌ではGitクライアント-サーバ計算機の構成が共にWindows，Linuxの場合について示す．

`Windows`

以下の図に示す構成でGitの導入を行う．

.. figure:: images/doc_git_structure-install-win.*

  WindowsにGitを導入する場合のシステム構成

1 *Gitサーバソフトウェアのインストール*

本作業はwsv計算機上で行うこと．

\(1\) Gitサーバソフトウェアのインストーラのダウンロード

* Internet Explorer等のWebブラウザを利用し，Gitのサーバソフトウェアであるmsysgitをダウンロードするhttp://code.google.com/p/msysgit/downloads/listの画面を開く．
* Git-1.7.8-preview20111206.exe(2011年12月執筆時点)のリンクをクリックする．

.. figure:: images/doc_git_install-1.*

  Gitクライアントソフトウェアのインストール(1/9)

\(2\) Gitサーバソフトウェアのインストール

* ダウンロードした「Git-1.7.8-preview20111206.exe」を実行する．
* 支持に従ってインストールを行う．なお，インストール画面は以下の通り．

.. figure:: images/doc_git_install-2.*

  Gitクライアントソフトウェアのインストール(2/9)

.. figure:: images/doc_git_install-3.*

  Gitクライアントソフトウェアのインストール(3/9)

.. figure:: images/doc_git_install-4.*

  Gitクライアントソフトウェアのインストール(4/9)

.. figure:: images/doc_git_install-5.*

  Gitクライアントソフトウェアのインストール(5/9)

.. figure:: images/doc_git_install-6.*

  Gitクライアントソフトウェアのインストール(6/9)

.. figure:: images/doc_git_install-7.*

  Gitクライアントソフトウェアのインストール(7/9)

.. figure:: images/doc_git_install-8.*

  Gitクライアントソフトウェアのインストール(8/9)

.. figure:: images/doc_git_install-9.*

  Gitクライアントソフトウェアのインストール(9/9)

..

2 *Gitクライアントソフトウェアのインストール*

本作業はwcl計算機上で行うこと．
なお，作業手順は「1 Git サーバソフトウェアのインストール」と同様．

`Linux`

以下の図に示す構成でGitの導入を行う．

.. figure:: images/doc_git_structure-install-linux.*

  LinuxにGitを導入する場合のシステム構成

1 *Gitサーバソフトウェアのインストール*

本作業はwsv計算機上で行うこと．

\(1\) Gitサーバソフトウェアのインストール

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
   0 個のパッケージを更新， 4 個を新たにインストール， 0 個を削除予定，206 個が更新されていない．
   6,315kB のアーカイブを取得する必要があります． 展開後に 13.1MB のディスク領域が新たに消費されます．
   先に進みますか? [Y/n/?] Y
   拡張状態情報を書き込んでいます... 完了
   取得:1 http://jp.archive.ubuntu.com/ubuntu/ lucid/main liberror-perl 0.17-1 [23.8kB]
   取得:2 http://jp.archive.ubuntu.com/ubuntu/ lucid/main libdigest-sha1-perl 2.12-1build1 [26.7kB]
   取得:3 http://jp.archive.ubuntu.com/ubuntu/ lucid-updates/main git-core 1:1.7.0.4-1ubuntu0.2 [6,143kB]
   取得:4 http://jp.archive.ubuntu.com/ubuntu/ lucid/main patch 2.6-2ubuntu1 [121kB]
   6,315kB を 15s 秒でダウンロードしました (406kB/s)
   未選択パッケージ liberror-perl を選択しています．
   (データベースを読み込んでいます ... 現在 156889 個のファイルとディレクトリがインストールされています．)
   (.../liberror-perl_0.17-1_all.deb から) liberror-perl を展開しています...
   未選択パッケージ libdigest-sha1-perl を選択しています．
   (.../libdigest-sha1-perl_2.12-1build1_amd64.deb から) libdigest-sha1-perl を展開しています...
   未選択パッケージ git-core を選択しています．
   (.../git-core_1%3a1.7.0.4-1ubuntu0.2_amd64.deb から) git-core を展開しています...
   未選択パッケージ patch を選択しています．
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

2 *Gitクライアントソフトウェアのインストール*

本作業はwcl計算機上で行うこと．
なお，作業手順は「1 Git サーバソフトウェアのインストール」と同様．


*利用例*

Gitの利用例(Windows，Linux)を以降に示す．

`Windows`

\(1\) Git利用例

* Gitユーザの登録

  * Git Bashの実行
  * Gitユーザの登録

.. code-block:: bash

   git config --global user.name "testUser"

* ローカル1リポジトリディレクトリの作成

  * Git Bashを実行し，Git Bash画面から「C:\gitrep\」フォルダ，リポジトリの作成を行う．

.. code-block:: bash

   mkdir /c/glocal1
   cd /c/glocal1
   git init

* 中央リポジトリディレクトリの作成

  * Git Bashを実行し，Git Bash画面から「C:\gitrep\」フォルダ，リポジトリの作成を行う．

.. code-block:: bash

   mkdir /c/gcenter
   cd /c/gcenter
   git --bare init

* ローカル1リポジトリにファイルのコミット

  * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   cd /c/glocal1
   touch first
   git add first
   git commit first -m "first commit"

* ローカル1リポジトリ(master)を中央リポジトリ(origin)にプッシュ

  * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   cd /c/glocal1
   git remote add origin /c/gcenter
   git push origin master

* 中央リポジトリ(origin)の作業コピーディレクトリ(ローカル2リポジトリ)を作成する．

  * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   mkdir /c/glocal2
   cd /c/glocal2
   git clone /c/gcenter

* ローカル2リポジトリに追加したファイルをローカル1リポジトリに反映

  * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   cd /c/glocal2/gcenter
   touch second
   git add second
   git commit second -m "second commit"
   git push origin master 

   cd /c/glocal1
   git pull origin master

..

`Linux`

\(1\) Git利用例

* Gitユーザの登録
.. code-block:: bash

   git config --global user.name "testUser"

* ローカル1リポジトリディレクトリの作成

  * /home/p1414/glocal1というリポジトリの作成を行う．

.. code-block:: bash

   mkdir /home/p1414/glocal1
   cd /home/p1414/glocal1
   git init

* 中央リポジトリディレクトリの作成

  * /home/p1414/gcenterというリポジトリの作成を行う．

.. code-block:: bash

   mkdir /home/p1414/gcenter
   cd /home/p1414/gcenter
   git --bare init

* ローカル1リポジトリにファイルのコミット

.. code-block:: bash

   cd /home/p1414/glocal1
   touch first
   git add first
   git commit first -m "first commit"

* ローカル1リポジトリ(master)を中央リポジトリ(origin)にプッシュ

.. code-block:: bash

   cd /home/p1414/glocal1
   git remote add origin /home/p1414/gcenter
   git push origin master

* 中央リポジトリ(origin)の作業コピーディレクトリ(ローカル2リポジトリ)を作成する．

  * Git Bashを起動し，以下を実行する．

.. code-block:: bash

   mkdir /home/p1414/glocal2
   cd /home/p1414/glocal2
   git clone /home/p1414/gcenter

* ローカル2リポジトリに追加したファイルをローカル1リポジトリに反映

.. code-block:: bash

   cd /home/p1414/glocal2/gcenter
   touch second
   git add second
   git commit second -m "second commit"
   git push origin master 

   cd /home/p1414/glocal1
   git pull origin master

Sourceforge.JP
^^^^^^^^^^^^^^

*概要*

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
--------------------------

テストの必要性
^^^^^^^^^^^^^^

ソフトウェア開発におけるテストの必要性について示す．
ソフトウェア開発は，「設計」→「実装」→「試験」→「利用」の手順で行われる．
ソフトウェアを利用していると，不具合，使い勝手が悪い，追加の機能が欲しいなどの理由から
再度上記の手順を繰り返す場合が多い．その手順を繰り返していくと徐々にソフトウェアの
規模が大きくなり，修正したプログラムが他のプログラムに与える影響を把握することが難しくなっていく．
結果として，修正したプログラムの問題により，他のプログラムが正常に動かなくなることもある．

よって，ソフトウェアの変更を行う際には，ソフトウェアの品質を維持するために変更した部分のみならず，変更していない部分のテストも行う必要がある．

ここで，継続的インテグレーション（Continuous Integration，CIと略すこともある）と呼ぶソフトウェア開発手法について説明する．
継続的インテグレーションとは，ビルド(コンパイル)・テスト・インスペクションなどを自動化し，これらの作業を1日に何度も 繰り返すことで，ビルド，テストの失敗を早期に発見し，ソフトウェアの品質維持，納期短縮を行うためのソフトウェア・エンジニアリングの習慣の集合である．

この開発手法によるソフトウェア開発を行うことで以下のようなメリットを受けることができる．

* 自動でビルド，テスト，インスペクションが行われているため，開発したソフトウェアに対する品質を継続的に維持できる．(品質維持)
* 追加・変更したソフトウェアに問題があった場合，早期にその問題を発見できる．早期に発見できると問題の原因特定が比較的容易に行えることが多く，結果として開発期間を短くすることができる．また，機械による作業によるケアレスミスを防ぐことができる．(納期短縮)

以降では，Jenkinsと呼ぶ，継続的インテグレーションを実践するためのソフトウェアについて説明する．

Jenkins
^^^^^^^

*概要*

Jenkinsとは，継続的インテグレーションのためのソフトウェアである．
Jenkinsはソフトウェアのビルド，テスト，インスペクションを継続的に行うための仕組みを持ち，エラーが発生した場合，ユーザに通知したりすることができる．
 
Jenkinsの特徴を以下に示す．

* インストールが容易．
* Javaで記述されているが，Java以外の言語でもCIを実践できる．
* 設定が容易．XMLなどのファイルを修正する必要はない．
* RSS/メールでビルド結果(成功，失敗)を通知することができる．
* ビルド結果等を可視化するためのレポート機能を持つ．
* Subversion，Git，Mercurial，Bazaarなどのソースコード管理ツールと連携し，自動で最新のテスト対象のソースを取得することができる．
* JUnit(単体試験フレームワーク)，Selenium(Webアプリケーションテストフレームワーク)，FindBugs(静的解析ツール)，JavaNCSS(メトリクスツール)，Emma/Cobertura(テストカバレッジ取得ツール)，CheckStyle(コーディング規約チェックツール)などをJenkins用のプラグインを通して利用することができる．(全てJava言語用)
* プラグインを開発し，Jenkinsの機能を拡張することができる．
* オープンソースで公開されており，ライセンスはMIT Licenseとなっている．

また，Jenkinsを利用する場合のシステム構成例を以下に示す．

.. figure:: images/doc-jenkins-structure.*

  Jenkinsを利用する場合のシステム構成


*導入*

Jenkinsは様々な環境に導入することができるが，本誌ではLinuxに導入する手順について示す．
また，Jenkinsを利用するためには，Apache/Tomcat等のWebアプリケーションサーバを用意する必要があるが，本誌ではこれらの
Webアプリケーションサーバを利用せず，Jenkinsに同胞されているWinstoneという軽量Servletコンテナを利用する．
導入時の構成を以下に示す。


      .. csv-table:: Sphinxのプロジェクト情報(Linux)
         :header: "項目", "内容"
         :widths: 20, 20

         "継続的インテグレーションシステム","Jenkins 1.441 2011年12月1日時点の最新版"
         "Webアプリケーションサーバ","Winstone"

..
..


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

2 *Jenkinsの起動/停止*

\(1\) 起動

  .. code-block:: bash
    :linenos:

    sudo /etc/init.d/jenkins start

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

..

\(2\) 停止

  .. code-block:: bash
    :linenos:

    sudo /etc/init.d/jenkins stop


Jenkinsの利用例
^^^^^^^^^^^^^^^

*概要*

以降にJenkinsの利用例を示す．より具体的な利用例を示すために，対象とするソースコードは
東京大学情報システム工学研究室(略称 JSK)の「RTM-ROS 相互運用プロジェクト」で作成したソースコードを利用する．

「RTM-ROS 相互運用プロジェクト」で作成したソースコードは以下のような機能を持つ．

.. todo:: ソースコードの機能例について記述する

ソースコードは，Subversionにより公開されており，また，ソースコード内には，ソースコードをJenkinsによりビルド，テストするためのソースも含まれているため，
読者の開発環境においても，以降の手順を踏むことで「RTM-ROS 相互運用プロジェクト」で作成したソースコードをJenkinsを利用して，ビルド・テストできる．

なお，RTM-ROS 相互運用プロジェクトの内容，研究成果については，以下のサイトで公開している．

http://code.google.com/p/rtm-ros-robotics/


*開発環境*

「RTM-ROS 相互運用プロジェクト」で開発しているソースコードをJenkinsでビルド・テストするための開発環境を示す．

  絵を描く

.. figure:: images/doc_jenkins_example_structure.*

  Jenkinsの利用構成

      .. csv-table:: Sphinxのプロジェクト情報(Linux)
         :header: "項目", "内容"
         :widths: 20, 20

         "OS","Ubuntu 10.04"
         "継続的インテグレーション","Jenkins version 1.441"
         "ビルドシステム","bash"
         "ソースコード管理システム","Subversion(Google Codeで実行)"
         "Jenkins実行ユーザ","jenkins"

*事前準備*

「RTM-ROS 相互運用プロジェクト」で作成したソースコードをJenkinsから利用する前に，以下の準備を行う必要がある．

 * ROSのインストール
 * Eclipseのインストール&設定
 * HRP4Cのモデルファイルの取得
 * Jenkinsの起動

\(\1) ROSのインストール

 ROSのインストール手順については，以下のサイトを参照すること．
 http://code.google.com/p/rtm-ros-robotics/wiki/ROS_Install

\(\2) Eclipseのインストール&設定

 Eclipseのインストール&設定手順については，以下のサイトを参照すること．
 http://code.google.com/p/rtm-ros-robotics/wiki/ROS_English#Setup_Eclipse

\(\3) HRP4Cのモデルファイルの取得

 HRP4Cとは，独立行政法人産業技術総合研究所により，エンターテインメント産業への応用を主な目的として開発されたサイバネティックヒューマンである．
 このHRP4Cのモデルファイルを使用するため，以下の手順により，HRP4Cのモデルファイルを取得する．

 1. 以下のページをブラウザにより開き，「HRP-4C外装付きモデルファイル」の「ダウンロード」リンクをクリックする．
  http://unit.aist.go.jp/is/humanoid/hrp-4c/hrp-4c.html

.. figure:: images/doc_hrp4c_download-1.*

  HRP4Cモデルのダウンロード画面(1/2)

 2. 「HRP-4C外装付きモデルファイル使用同意書」ページの下の「お名前」，「ご所属」，「メールアドレス」を入力し，「送信」ボタンを押す．

.. figure:: images/doc_hrp4c_download-2.*

  HRP4Cモデルのダウンロード画面(2/2)

 3. 上記で指定したメールアドレスにダウンロード先のリンクが記述されているため，そのダウンロード先から「HRP-4C.zip」を取得する．
 4. 取得した「HRP-4C.zip」をサーバのDownloadディレクトリにコピーする．

.. todo:: 上記の内容を修正すること．

\(\4) Jenkinsの起動

 「2 *Jenkinsの起動/停止*」の\(1\) 起動を実行する．


