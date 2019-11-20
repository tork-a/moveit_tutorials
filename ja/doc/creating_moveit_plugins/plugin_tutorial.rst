Creating Moveit Plugins
========================
`このページ <http://wiki.ros.org/pluginlib>`_ ではROS general にPluginを追加する方法を説明します。Pluginを作成する上で必要な要素は、BaseとPlugin classです。Plugin Classは base classを継承し、仮想関数をオーバーライドします。このチュートリアルは異なる３種類のプラグイン（motion planner, controller manager, constraint sampler plugin)が含まれます。


Motion Planner Plugin
----------------------
この章では、motion plannerをMoveitのプラグインとして追加する方法を説明します。Moveitのこのbase classは全ての新しいプラグインが継承する ``planning_interface`` です。ここでは例として、joint spaceの2つの状態のモーションプランを生成するlinear interpolation planner (lerp)を作成します。このプランナーは新しいプランナーを追加する上で必要な基本要素を全て含んでいるため、スタートポイントとして使うことができます。下記の図は新しいプランナーをMoveitに追加することに関連する、各class間の関係の概要を示しています。

.. image:: lerp_motion_planner/lerp_planner.png

まず、``moveit_tutorials`` packageの中にプラグインを作成します。プラグインクラス ``lerp``を作成するためには、フォルダ ``src`` の中に、 ``lerp_planner_manager.cpp` `という名前のファイルを作成します。このファイルの中で、``LERPPlanPlannerManager`` は ``planning_interface`` から ``PlannerManager`` クラスの関数をオーバーライドします。このファイルの最後で、 ``LERPPlanPlannerManager`` クラスをプラグインとして登録する必要があります。これは ``class_loader`` から、``CLASS_LOADER_REGISTER_CLASS`` macroによって実行されます。: ::

  CLASS_LOADER_REGISTER_CLASS(emptyplan_interface::EmptyPlanPlannerManager, planning_interface::PlannerManager);

``LERPPlanningContext``は``PlanningContext``の関数をオーバーライドするクラスです。このクラスはプランナーが問題を解き結果を返すsolve functionを含みます。solve functionはプランナーのソースコードから多くのクラスを必要とする可能性があるため、実際のプランナーのsolve methodが実際に実装される``lerp_interface``を呼び出す他のクラスを作る方が効率的です。基本的に、このclassは新しいモーションプランアルゴリズムのためのエントリーポイントです。このsolve functionの応答は``LERPPlanningContext``クラスで``planning_interface::MotionPlanDetailedResponse``タイプに変換される``moveit_msgs::MotionPlanDetailedResponse``タイプで準備されます。

更に、``PlannerConfigurationSettings``はプランナーにパラメータを渡すためにも使われます。パラメータを渡す他の方法にはyamlファイルからパラメータを読み込むROSパラメータサーバーを使う方法があります。このチュートリアルでは、solve functionがCallされた時にロードされる``lerp_interface``に記載されている``num_steps``だけを持つ``panda_moveit_config``パッケージの``lerp_planning.yaml``を使用します。

Export the plugin
^^^^^^^^^^^^^^^^^^

最初に、プラグインをROSのツールチェーンで利用できるようにする必要があります。そのためには、plugin description xml file(``emptyplan_interface_plugin_description.xml``)を作成する必要があります。このファイルには、ライブラリタグがオプション設定と一緒に記載されています。: ::

  <library  path="libmoveit_emptyplan_planner_plugin">
    <class name="emptyplan_interface/EmptyPlanPlanner" type="emptyplan_interface::EmptyPlanPlannerManager" base_class_type="planning_interface::PlannerManager">
     <description>
     </description>
    </class>
  </library>

そして、プラグインをエクスポートするには、上のxmlファイルのアドレスとpackage.xmlファイルの``export``タグを使います。: ::

 <export>
    <moveit_core plugin="${prefix}/emptyplan_interface_plugin_description.xml"/>
 </export>

注：タグの名前``moveit_core``はbase classである``planning_interface``が存在するパッケージの名前と同じです。

Check the plugin
^^^^^^^^^^^^^^^^^

次のコマンドによって、新しいプラグインが正しく作成され、エクスポートされたか確認できます。: ::

  rospack plugins --attrib=plugin moveit_core

この結果は``moveit_planners_lerp``にplugin description xmlファイルのアドレスと一緒に含まれる必要があります。: ::

  moveit_tutorials <ros_workspace>/src/moveit_tutorials/creating_moveit_plugins/lerp_motion_planner/lerp_interface_plugin_description.xml

Plugin usage
^^^^^^^^^^^^^

この章では、先ほど作成したlerp plannerのロードと使い方を説明します。そのためには、``lerp_example.cpp``というROSノードを作成する必要があります。最初のステップは下記のコードでリクエストされたplanning groupとthe planning sceneに関係するロボットのジョイントのグループとその状態の取得です。: ::

  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

次のステップはpluinglibを使用するプランナーをロードし、パラメータ``planner_plugin_name``を先ほど作成した物に設定することです。: ::

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name =  "lerp_interface/LERPPlanner";
    node_handle.setParam("planning_plugin", planner_plugin_name);

プランナーをロードしたら、モーションプラニングのスタートとゴールの状態を設定します。スタート状態は、``req.start_state``で設定されたロボットの現在の状態です。反対に、ゴール制約はゴールの状態とジョイントモデルグループを使用して``moveit_msgs::Constraints``を作ることによって設定されます。この制約は``req.goal_constraint``に与えられます。次のコードはそれらを実施する方法を示しています。: ::

  // Get the joint values of the start state and set them in request.start_state
  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  // Goal constraint
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { 0.8, 0.7, 1, 1.3, 1.9, 2.2, 3 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

これまでプランナーをロードし、スタートとゴールの状態を作成しましたが、まだモーションプラニングの問題を解いていません。スタートとゴールの状態について与えられた情報によって、ジョイントの状態の中でモーションプラニング問題は、``PlanningContext``インスタンスを作成し、そのsolve functionをCallすることで実行されます。このsolve functionに渡される応答は``planning_interface::MotionPlanResponse``タイプということを覚えといてください。: ::

    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

最後に、このノードを実行するため、launchフォルダーでlerp_example.launchをroslaunchする必要があります。このlaunchファイルは``lerp``をPlannerの名前として渡すことによって、パッケージ``panda_moveit_config``の``demo.launch``を立ち上げます。その後、``lerp_example``が立ち上がり、lerpのパラメータをROS Parameter Serverに設定するため``lerp_planning.yaml``はロードされます。

Example Controller Manager Plugin
----------------------------------

MoveIt controller managers（なぜか誤った名称）は低い階層のカスタム コントローラのインターフェースです。これはコントローラインターフェースと考えた方が良いです。ほとんどのユースケースで、既にROS actionsを供給している場合、ロボットコントローラがFollowJointTrajectoryを実行にはincludeの:moveit_codedir:`MoveItSimpleControllerManager <moveit_plugins/moveit_simple_controller_manager>`が適当です。ros_controlを使用する場合も、includeの:moveit_codedir:`MoveItRosControlInterface <moveit_plugins/moveit_ros_control_interface>`が理想的です。

しかしながら、一部のアプリケーションでは、よりカスタムされたコントローラーマネージャーが必要な場合があります。カスタムコントローラーマネージャーを起動するためのテンプレートの例をここに示します。:codedir:`<controller_configuration/src/moveit_controller_manager_example.cpp>`

Example Constraint Sampler Plugin
----------------------------------

* ``ROBOT_moveit_plugins``パッケージを作成し、その中に``ROBOT_moveit_plugins``プラグインのためのサブフォルダを作成してください。そして、``ROBOT_moveit_plugins/ROBOT_moveit_constraint_sampler_plugin``から提供されたテンプレートを修正してください。

* ``ROBOT_moveit_config/launch/move_group.launch``ファイルの``<node name="move_group">``の中にパラメータを追加してください。: ::

  <param name="constraint_samplers" value="ROBOT_moveit_constraint_sampler/ROBOTConstraintSamplerAllocator"/>

* move_groupを立ち上げると、それが新しい制約samplerのデフォルトになります。

