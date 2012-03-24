= RigidBox : A Small Library for 3D Rigid Body Physics Tutorial

3D剛体物理の実装方法を紹介する小さなライブラリです。

                                             Last Update: Mar 24, 2012
                                                   Since: Feb 23, 2011
                                                   by vaiorabbit

* はじめに
* 特徴
* 確認済みの動作環境
* ビルド方法
* デモの実行
* ソースコードの読みかた
* クレジット
* ライセンス


== はじめに

剛体物理エンジンの利用がゲーム製作者の間で人気を集めています。
最近になって利用方法に関する書籍も複数登場しました。しかし、
剛体物理エンジンの「制作」に関する情報は少ないようです。

興味本位でオープンソースの剛体物理エンジンをのぞいてみても、
概要を理解するのは一苦労だと思います。高度な機能が大量に含まれていて、
本質的な部分が見えづらくなっているのが普通です。

その剛体物理エンジンの「本質的な部分」とは、大きく次のように
分けることができます：

1. 物体同士の衝突について調べる「衝突検出 (Collision Detection)」
2. 衝突後の跳ね返り方を計算する「衝突応答 (Collision Response)」
3. 力や速度から次の位置を決める「積分 (Integration)」

この処理のループを "Update()" や "StepSimulation()" といった
関数の下で実行してくれるライブラリ、それが剛体物理エンジンです。

RigidBoxライブラリは以上をできる限り手短に実演することで、
剛体物理エンジンの「本質的な部分」を最短距離で理解してもらうことを
目的としています。



== 特徴

三次元空間内に配置した十数個程度の直方体同士について、
その物理的な挙動をシミュレートすることができます。

記述を簡潔にするため、以下の方針で実装を行っています：

【箱(直方体)以外の形状を扱いません】

  * 衝突検出処理には「直方体同士の衝突検出」以外のコードがないため簡潔です。
  * 箱の傾けかたが理解できれば満足ですよね?
  * 球すら扱いません。簡単なので実装後の感動がありません。
  * 平面も扱いません。床や壁は大きな直方体で代用できます。

【積分：クォータニオンを使いません】

  * 必要な知識も最小に。3x3行列で代用できるのでそうしました。
    ※高速化および計算誤差対策から実用上は必須です。

【衝突検出：高度な衝突処理は行いません】

  * 必要な知識も最小に。衝突検出は剛体同士の総当たりで行います。
    ※計算の高速化のためブロードフェーズ衝突検出が実用上は必須です。

【衝突応答：高度なソルバーはありません】

  * 数式通りの実装をなるべく素直に利用します。反復解法は利用しません。
  * 簡潔さのためなら安定性は無視します。

【簡潔な記述 > 効率的な記述】

  * 数式通りの実装に近付けるため、数学関数はオペレーターの
    オーバーロードを多用しています。
  * STLでデータの保持/管理を簡略化しています。

以上、物理エンジンとしての実用性を無視することで、ライブラリ全体を
理解しやすいコード量に抑えることに注力しています。

※[2011-07-15]まだコメントをほとんど入れていない現時点での行数は
  およそ1,800行となっています。

  $ wc -l source/*cpp include/RigidBox/*h
     348 source/rbCollision.cpp
     163 source/rbEnvironment.cpp
     168 source/rbRigidBody.cpp
      63 source/rbSolver.cpp
      12 include/RigidBox/RigidBox.h
      51 include/RigidBox/rbCollision.h
      65 include/RigidBox/rbEnvironment.h
     537 include/RigidBox/rbMath.h
     310 include/RigidBox/rbRigidBody.h
      23 include/RigidBox/rbSolver.h
      70 include/RigidBox/rbTypes.h
    1810 total



== 確認済みの動作環境

* Mac OS X
  * Version  : Snow Leopard (10.6)
  * Compiler : Xcode 4

* Windows
  * Version  : Windows Vista
  * Compiler : Visual C++ 2010 + Windows SDK v7.0A

* Linux
  * Version  : Ubuntu 11.04
  * Compiler : GCC 4.5



== ビルド方法

=== 必要なもの

==== CMake

※Visual Studio 2010 および Xcode 4 を利用する場合は不要です。
  build ディレクトリにプロジェクトファイルを同梱しています。

* http://www.cmake.org/
* 各自の環境に合わせたビルドスクリプトの生成に利用します。

コマンドライン上で build ディレクトリへ移動し、以下のコマンドを実行してください。

* Windows
  * > cmake -G "Visual Sutdio 2010" ..
    Visual Studio ソリューションファイル (RigidBox.sln) が生成されます。

* Mac OS X
  * $ cmake -G "Xcode" ..
    Xcode プロジェクト (RigidBox.xcodeproj) が生成されます。

* Linux (Ubuntu)
  * $ cmake -G "Unix Makefiles" ..
    Makefile が生成されます。

==== OpenGL

* Windows
  * gl.h : Windows SDK をインストールしてください。
    http://www.microsoft.com/download/en/details.aspx?displaylang=en&id=8279
  * glext.h : OpenGL.org より入手してください。
    http://www.opengl.org/registry/api/glext.h
      
* Mac OS X
  * OS に標準で同梱されています。

* Linux (Ubuntu)
  * Synaptic などで OpenGL 関連の dev パッケージが入手できます。

==== GLUT

* Windows
  * FreeGLUT で動作確認しています。
    http://freeglut.sourceforge.net/
    
* Mac OS X
  * OS に標準で同梱されています。

* Linux (Ubuntu)
  * Synaptic などで GLUT 関連の dev パッケージが入手できます。


=== ライブラリのビルド

* Windows
  * build ディレクトリ以下にある RigidBox.sln を利用します。
  * RigidBox.lib が生成されます。場所は lib/Debug および lib/Release です。

* Mac OS X
  * build ディレクトリ以下にある RigidBox.xcodeproj を利用します。
  * RigidBox.dylib が生成されます。場所はユーザーによって異なります。
    例) /Users/[ユーザー名]/Library/Developer/Xcode/DerivedData/ [改行] 
            RigidBoxDemo-egcoaigiqbgfwabxxwpufqtasprk/Build/Products/Debug/RigidBox.dylib

* Linux (Ubuntu)
  * build ディレクトリで make を実行してください。
  * libRigidBox.a が生成されます。場所は lib/ です。



== デモの実行

* demo ディレクトリにある CollisionDemo および StackDemo を
  参照してください。

* DemoViewer は OpenGL/GLUT を利用したデモプログラム用の
  フレームワークです。

* デモプログラムのビルド方法はライブラリの手順に準じます。

  * ビルドスクリプトの生成は demo/build ディレクトリで
    cmake を実行することで行います。

  * Windows
    * build ディレクトリ以下にある RigidBoxDemo.sln を利用します。
      CollisionDemo または StackDemo がスタートアッププロジェクトに
      指定されていることを確認してください。

  * Mac OS X
    * build ディレクトリ以下にある RigidBoxDemo.xcworkspace を利用します。
      現在選択中の Scheme が CollisionDemo または StackDemo と
      なっていることを確認してください。

  * Linux (Ubuntu)
    * build ディレクトリで make を実行してください。
      実行ファイルは build/CollisionDemo/CollisionDemo などです。



== ソースコードの読みかた

[2012-03-24] 下記 URL でドキュメントを整備中です。

  http://code.google.com/p/rigidbox/wiki/ImplementationNotes
  http://code.google.com/p/rigidbox/w/list

前提知識は以下の通りです：

* 数学
  * 3次元ベクトルと3x3行列の取り扱い方法を理解している
  * ワールド空間/オブジェクト空間の区別がつく

* 物理
  *  : 高校物理程度 (質点(大きさのない物体)の運動や衝突について勉強したことがある)

* C++
  * 言語仕様 : クラス、オペレーターのオーバーロードについて理解している
  * ライブラリ : STL、特に vector とイテレーターを利用したことがある



== クレジット

* vaiorabbit (http://twitter.com/vaiorabbit)


 
== ライセンス

zlib/libpng ライセンスです。LICENSE.txt を参照してください。
