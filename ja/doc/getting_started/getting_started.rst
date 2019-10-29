はじめに
===============

本チュートリアルでは，MoveItをインストールし，チュートリアルを実行するためのワークスペースを構築します．

ROSとCatkinをインストールする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ からROS Melodic のインストールを行いましょう.
ROSのインストールではミスが起きがちです．もし下記手順の中でうまく行かない部分がある場合にはリンクに戻り，ROSがきちんとインストールされていることを確認してください．

ROSのインストールが完了したら，パッケージのアップデートを行いましょう． ::

  rosdep update
  sudo apt-get update
  sudo apt-get dist-upgrade

次に，ROSのビルドシステムである `catkin <http://wiki.ros.org/catkin>`_ のインストールを行います． ::

  sudo apt-get install ros-melodic-catkin python-catkin-tools

Catkinワークスペースの構築とMoveItのソースコードのダウンロード
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
チュートリアルはMoveItのmasterブランチのソースコードに基づいており，ソースコードからのビルドが必要です．
まずは `catkin <http://wiki.ros.org/catkin>`_ ワークスペースのセットアップから行いましょう． ::

  mkdir -p ~/ws_moveit/src
  cd ~/ws_moveit/src

  wstool init .
  wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
  wstool update -t .

ソースコード例のダウンロード
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
これから先のチュートリアルを行っていくにあたり， **ROBOT_moveit_config** パッケージ（ **ROBOT** の部分はそれぞれのロボットの名前が入ります）が必要になります．デフォルトのデモ用ロボットとして Franka Emika社の「Panda」を利用します．もし **panda_moveit_config** パッケージを利用するなら，ソースコードからのインストールをおすすめします．
作成した `catkin <http://wiki.ros.org/catkin>`_ ワークスペース下に ``panda_moveit_config`` パッケージと本チュートリアルをダウンロードしましょう． ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git -b master
  git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

.. note:: ここでは予め作成しておいた ``panda_moveit_config`` パッケージを利用しますが， 後ほどオリジナルロボット用のパッケージの作成方法を `MoveItセットアップアシスタントチュートリアル <../setup_assistant/setup_assistant_tutorial.html>`_ にて紹介します.

Catkinワークスペースをビルドする
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
まず，下記コマンドでワークスペース内のパッケージが依存するDebianパッケージをインストールしましょう． ::

  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro melodic

次に，catkinワークスペースの設定を行い，ビルドします． ::

  cd ~/ws_moveit
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build

最後にcatkinワークスペースへのPATH通しを行いましょう． ::

  source ~/ws_moveit/devel/setup.bash

オプション：下記コマンドを実行しておくと毎回端末を開く際にPATH通しをする必要がなくなります．::

   echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

.. note:: 上記の ``setup.bash`` を ``~/.bashrc`` で読み込ませるのは必須ではありません．特に，複数のcatkinワークスペースを使用している場合は行いません．なるべく簡単にセットアップを行うためにここでは紹介しました．

次のステップ
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RViz用のインタラクティブなモーションプランニングプラグインとロボットの可視化 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
