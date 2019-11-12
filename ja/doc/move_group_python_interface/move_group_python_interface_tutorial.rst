Move Group Python インタフェース
================================================
.. image:: move_group_python_interface.png
   :width: 700px

MoveItのユーザーインタフェースの利用する際の最も簡単な方法は，Pythonから"Move Group"インタフェースを使用する方法です．これらのラッパーはどんなユーザーにも必要と思われる，特に最終的なジョイントや姿勢の設定や動作計画，障害物のある環境構築，ロボットに物体を付け外し，とほとんどの制御の機能を提供してくれます．

こちらの動画 `YouTube video demo <https://youtu.be/3MA5ebXPLsc>`_ を観て，Move Group Python インタフェースで何ができるか確認しましょう!

準備
---------------
もしまだ済ましていなければ，まず `Getting Started <../getting_started/getting_started.html>`_ から始めてください．

RViz と MoveGroup ノードの起動
-----------------------------------------------------------------
Shellを２つ立ち上げ，一方はRVizを起動しすべて完了するまで待ちます: ::

  roslaunch panda_moveit_config demo.launch

もう一方のshellでは，``rosrun`` を用いてPythonコードを直接走らせます．またpythonスクリプトを実行可能にする必要があるかもしれません: ::

 rosrun moveit_tutorials move_group_python_interface_tutorial.py

今回のゴール
---------------
RViz上で, 下記のことができているか確認します:

一つ前の章の ``rosrun`` を走らせたターミナルのshellで *<enter>* を押して，下記の各々のステップに進んでください．
 #. ロボットの腕の動きが計画されており，ゴールジョイントまで移動すること
 #. ロボットのゴール姿勢までの経路が計画されていること
 #. ロボットがCartesianで経路が計画されていること
 #. ロボットが再度Cartesian経路で計画されていること
 #. ロボットがそのCartesian経路で実行されていること
 #. Pandaのエンドエフェクタの位置に立方体が構築されていること
 #. 立方体が取り付けられていることを示すために，その色が変わっていること
 #. 立方体が取り付けられたロボットがCartesian経路で計画/実行されていること
 #. 立方体が取り外されたことを示すために，色が変わっていること
 #. 立方体が消えていること

Pythonコードの中身
----------------------------------------------
アナウンス: 今回のpythonコードはまとめて :codedir:`チュートリアルのGitHubレポジトリはここ<move_group_python_interface/scripts/move_group_python_interface_tutorial.py>` で見られます．

.. tutorial-formatter:: ./scripts/move_group_python_interface_tutorial.py

Launch ファイル
---------------
今回のLaunchファイルはGitHubの :codedir:`ここ<move_group_python_interface/launch/move_group_python_interface_tutorial.launch>`
にあります．このチュートリアルで用いられたコードはMoveItのセットアップのうちの
``moveit_tutorials`` パッケージから走らせることができます．
