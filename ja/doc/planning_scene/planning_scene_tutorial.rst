プランニング環境
==================================

:planning_scene:`PlanningScene` クラスは衝突や拘束条件を確認するためのメインインタフェースを提供してくれます．
本チュートリアルでは，C++のインタフェースを用います．

はじめに
---------------
もしまだ済ましていなければ，まず `はじめに <../getting_started/getting_started.html>`_ から始めてください．

C++コード
------------------------
本チュートリアルで用いられたC++コードは， :codedir:`こちらのMoveIt GitHub project<planning_scene>` で見ることが出来ます．

.. tutorial-formatter:: ./src/planning_scene_tutorial.cpp

Launchファイル
-----------------------------
本チュートリアルで用いられたlaunchファイルは，GitHubの :codedir:`こちら <planning_scene/launch/planning_scene_tutorial.launch>` から見られます．本チュートリアルで用いられている全てのコードは，moveit_tutorialsパッケージからコンパイルできます．

コードを走らせる
-------------------------------
Roslaunchを用いることで，直接 ``moveit_tutorials`` からlaunchファイルのコードを走らせることができます: ::

  roslaunch moveit_tutorials planning_scene_tutorial.launch

今回のゴール
-------------------------

本チュートリアルでは，以下の形式で出力できることが目標です．ランダムな関節角を用いるため，数字が一致しなくても大丈夫です: ::

 ros.moveit_tutorials: Test 1: Current state is not in self collision
 ros.moveit_tutorials: Test 2: Current state is not in self collision
 ros.moveit_tutorials: Test 3: Current state is not in self collision
 ros.moveit_tutorials: Test 4: Current state is valid
 ros.moveit_tutorials: Test 5: Current state is in self collision
 ros.moveit_tutorials: Contact between: panda_leftfinger and panda_link1
 ros.moveit_tutorials: Contact between: panda_link1 and panda_rightfinger
 ros.moveit_tutorials: Test 6: Current state is not in self collision
 ros.moveit_tutorials: Test 7: Current state is not in self collision
 ros.moveit_tutorials: Test 8: Random state is not constrained
 ros.moveit_tutorials: Test 9: Random state is not constrained
 ros.moveit_tutorials: Test 10: Random state is not constrained
 ros.moveit_tutorials: Test 11: Random state is feasible
 ros.moveit_tutorials: Test 12: Random state is not valid

**Note:** ROSコンソールの出力形式が異なっていても安心してください．もし同じような形式にしたければ， `こちら <http://dav.ee/blog/notes/archives/898>`_ をご覧ください．
