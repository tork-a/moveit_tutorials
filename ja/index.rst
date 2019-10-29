MoveIt チュートリアル
============================

本チュートリアルでは，MoveIt Motion Planning Frameworkをすぐに使うための方法を紹介します．

.. image:: doc/quickstart_in_rviz/rviz_plugin_head.png
   :width: 700px

本チュートリアルでは，Franka Emika Panda をクイックスタートのデモ用ロボットとして使用します．このロボット以外にも，MoveItのセットアップが完了しているロボットであれば簡単に利用することができます．是非 `list of robots running MoveIt <http://moveit.ros.org/robots/>`_ のページをご覧頂き，どのロボットならMoveItを利用可能になっているのかをチェックしてみてください．仮にMoveItが利用可能なロボット一覧の中に無くても，「新規のロボットのセットアップ」セクションにてオリジナルのロボットでのセットアップ方法を紹介します．

MoveItとRVizから始める 
-------------------------------------
.. toctree::
   :maxdepth: 1

   doc/getting_started/getting_started
   doc/quickstart_in_rviz/quickstart_in_rviz_tutorial

MoveGroup - C++，PythonのROSラッパー
------------------------------------------
スクリプトからMoveItを動かす最も簡単な方法は ``move_group_interface`` を利用する方法です．このインターフェースは初心者にとって理想的であり，MoveItの多くの機能に簡単にアクセスすることができます．

.. toctree::
   :maxdepth: 1

   doc/move_group_interface/move_group_interface_tutorial
   doc/move_group_python_interface/move_group_python_interface_tutorial
   doc/moveit_commander_scripting/moveit_commander_scripting_tutorial

C++ APIから直接MoveItを使う
------------------------------------------
MoveItを用いてより複雑なアプリケーションを作る際には，MoveItのC++ APIが必要になるでしょう．また，C++ APIを利用することでROSの Service/Actionレイヤーをスキップし，大幅な速度向上にもつながるでしょう．

.. toctree::
   :maxdepth: 1

   doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial
   doc/planning_scene/planning_scene_tutorial
   doc/planning_scene_ros_api/planning_scene_ros_api_tutorial
   doc/motion_planning_api/motion_planning_api_tutorial
   doc/motion_planning_pipeline/motion_planning_pipeline_tutorial
   doc/creating_moveit_plugins/plugin_tutorial
   doc/visualizing_collisions/visualizing_collisions_tutorial
   doc/time_parameterization/time_parameterization_tutorial
   doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial
   doc/pick_place/pick_place_tutorial
   doc/moveit_grasps/moveit_grasps_tutorial
   doc/moveit_task_constructor/moveit_task_constructor_tutorial
   doc/subframes/subframes_tutorial          
              
新規のロボットのセットアップ
----------------------------
MoveItで新規のロボットとしてセットアップする前に，あなたのロボットがすでにセットアップされているかどうかを確認してみてください．（`list of robots running MoveIt <http://moveit.ros.org/robots/>`_ から確認できます．）リストに無い場合は，本セクションのチュートリアルに従ってセットアップしてください．（そして是非，MoveItのメーリングリストでセットアップの結果を共有してください．）

.. toctree::
   :maxdepth: 1

   doc/setup_assistant/setup_assistant_tutorial
   doc/urdf_srdf/urdf_srdf_tutorial
   doc/controller_configuration/controller_configuration_tutorial
   doc/perception_pipeline/perception_pipeline_tutorial
   doc/ikfast/ikfast_tutorial
   doc/trac_ik/trac_ik_tutorial

その他の設定
-----------------------
.. toctree::
   :maxdepth: 1

   doc/kinematics_configuration/kinematics_configuration_tutorial
   doc/custom_constraint_samplers/custom_constraint_samplers_tutorial
   doc/ompl_interface/ompl_interface_tutorial
   doc/chomp_planner/chomp_planner_tutorial
   doc/stomp_planner/stomp_planner_tutorial
   doc/trajopt_planner/trajopt_planner_tutorial
   doc/planning_adapters/planning_adapters_tutorial.rst

その他の機能
----------------------------

.. toctree::
   :maxdepth: 1

   doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial
   doc/arm_jogging/arm_jogging_tutorial
   doc/benchmarking/benchmarking_tutorial
   doc/tests/tests_tutorial

帰属
-----------
MoveItチュートリアルの主な貢献者（古い順）：
Sachin Chitta, Dave Hershberger, Acorn Pooley, Dave Coleman, Michael Gorner, Francisco Suarez, Mike Lautman．
是非一緒にドキュメントの更新をしましょう．あなたの名前を追加するのを楽しみにしています！

本チュートリアルは Franka Emika，PickNik の支援で2018年にメジャーアップデートを行いました．（`是非ブログもチェックしてみてください！ <http://moveit.ros.org/moveit!/ros/2018/02/26/tutorials-documentation-codesprint.html>`_）


.. image:: ./_static/franka_logo.png
   :width: 300px
