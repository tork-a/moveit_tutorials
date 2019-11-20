/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

// BEGIN_SUB_TUTORIAL stateFeasibilityTestExample
//
// ユーザ定義型の拘束条件は :planning_scene:`PlanningScene` クラスでも決めることが出来ます．
// これは ``setStateFeasibilityPredicate`` 関数を用いてコールバックを明示することで，行えます．
// ユーザが定義したコールバックはPandaロボットの ``panda_joint1`` の値が正か負か確かめる簡単な例がこちらです:
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}
// END_SUB_TUTORIAL

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std::size_t count = 0;
  // BEGIN_TUTORIAL
  //
  // 準備
  // ^^^^^^^^^^^^^^^
  //
  // :planning_scene:`PlanningScene` クラスは簡単に設定でき，
  // :moveit_core:`RobotModel` やURDF，SRDFを用いて構成されています．
  // しかしこれは，プランニング環境のインスタンスを生成するのに推奨されていない方法です．
  // :planning_scene_monitor:`PlanningSceneMonitor`
  // は，現在のロボットの関節やセンサのデータを扱っているプランニング環境の作成と維持をするのに推奨されている方法です
  // （これに関しては次のチュートリアルで詳しく扱います）．
  // 今回のチュートリアルでは，直接 :planning_scene:`PlanningScene` クラスをインスタンス化していますが，
  // このインスタンス化の方法は，描画の際にのみ使われます．

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // 衝突判定
  // ^^^^^^^^^^^^^^^^^^
  //
  // 自己衝突判定
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // まずロボットの現在の状態が *self-collision* の状態，
  // つまりロボットのパーツが干渉しあっている状態で
  // 構築されているかを確認します．
  // そのため，
  // :collision_detection_struct:`CollisionRequest` オブジェクトと
  // :collision_detection_struct:`CollisionResult` オブジェクトを構築し，
  // 衝突判定関数にパスを通します．ロボットが自己干渉しているかどうかは，
  // その結果によります．自己衝突判定は，ロボットの *unpadded* バージョン，
  // つまり新たにパディングされていないURDFから得られた衝突メッシュを直接扱います．

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 状態の変更
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // ロボットの今の状態に変更を加えてみましょう．
  // プランニング環境では，現在の状態を内部で維持してくれます．
  // それらの情報の参照や変更，そして新しく構築したロボットの衝突判定もできます．
  // 再度衝突判定をする際には，先に ``collision_result`` をクリアにしなければいけないので注意してください．

  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // グループでの判定
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // ここでは，Pandaロボットの手のみに衝突判定を行います，
  // つまり手とロボットの他の部位との衝突が発生しているのかどうかを確認します．
  // ``colision_request`` に ``hand`` というグループ名を加えることで，
  // 確認することが出来ます．

  collision_request.group_name = "hand";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 衝突情報の取得
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // まず，内部（自己）干渉が起こる任意の位置にPandaロボットの腕を動かします．
  // 直接調べることは可能ですが，
  // 任意で動かした腕の状態は限界関節角度を超えてしまっていることも
  // あるので，注意してください．

  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Test 4: Current state is "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  // ようやくコンフィグレーションされたPandaロボットの腕に関するどんな些細な衝突情報でも得られるようになりました．
  // ``colision_request`` の適切な場所を埋め，
  // 衝突を判定する最大数を指定することで，衝突情報を得ることが出来ます．

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  //

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }

  // 衝突判定除外マトリクスの変更
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // :collision_detection_class:`AllowedCollisionMatrix` (ACM)
  // は環境内のロボット自身や物体同士の衝突を無視するように指示するための
  // メカニズムを提供してくれます．
  // 衝突判定システムに上記のリンク同士の全ての衝突を無視するように，
  // つまり実際はリンクが衝突していたとしても，システム上では
  // これらのリンクの衝突を無視し，そのロボットの状態は衝突していないと返すように命令します．
  //
  // 今回の例では，衝突判定除外マトリクスや現在の状態のコピーの仕方や
  // 衝突判定関数にそれらを渡す方法を記してあります．

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 全衝突判定
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 先程まで自己衝突判定のみ行ってきましたが，
  // 自己衝突判定と（現在はなにもありませんが）環境内衝突判定の２つを行ってくれる ``checkCollision`` 関数を代わりに使っていきます．
  // この関数は全ての衝突判定をしてくれるので，プランナ内でよく用いられます．
  // 環境内の衝突判定ではパディングされたロボットが用いられます．
  // パディングとは，環境内でロボットを障害物と一定の距離を保ってくれることを指します．
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 拘束条件の確認
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // :planning_scene:`PlanningScene` クラスは，拘束条件を確認するための便利な関数を有しています．
  // 拘束条件には２つのタイプがあります:
  // (a) 
  // :kinematic_constraints:`KinematicConstraint` から選ばれた拘束条件:
  // 例えば :kinematic_constraints:`JointConstraint` や，
  // :kinematic_constraints:`PositionConstraint` ，
  // :kinematic_constraints:`OrientationConstraint` ，
  // :kinematic_constraints:`VisibilityConstraint` もしくは (b) ユーザが
  // コールバックをもとに決定した拘束条件です．
  // まずは単純な :kinematic_constraints:`KinematicConstraint` 条件下の例から見ていきましょう．
  //
  // 運動拘束の確認
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // まず，Pandaロボットのグループである ``panda_arm`` のエンドエフェクタの単純な位置や方向の拘束を定義します．
  // 拘束条件を満たす便利な関数の使用を記しておきます．
  // （これらの関数は， *moveit_core* にある *kinematic_constraints* ディレクトリの :moveit_core_files:`utils.h<utils_8h>` で見つけられます．）

  std::string end_effector_name = joint_model_group->getLinkModelNames().back();

  geometry_msgs::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.3;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 0.5;
  desired_pose.header.frame_id = "panda_link0";
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

  // ``isStateConstrained`` 関数を ``PlanningScene`` クラスで用いると，
  // 拘束条件下での状態を確認できます．

  copied_state.setToRandomPositions();
  copied_state.update();
  bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
  ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

  // 拘束条件を確認するのにもっと効率的な方法があります．例えば，
  // プランナの中で，何度も何度も同じ拘束条件を確認したいときなどに効率的です．
  // まず，ROS Constraintsメッセージを前処理したり，
  // 拘束に処理するための準備をする ``KinematicConstraintSet`` を構築します．

  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
  kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
  bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

  // ``KinematicConstraintSet`` クラスを用いれば直接先程の作業を行えます．

  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
      kinematic_constraint_set.decide(copied_state);
  ROS_INFO_STREAM("Test 10: Random state is "
                  << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

  // 拘束条件の設定
  // ~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // CALL_SUB_TUTORIAL stateFeasibilityTestExample

  // ``isStateFeasible`` が呼ばれると，ユーザの設定したコールバックが
  // 呼ばれます．

  planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
  bool state_feasible = planning_scene.isStateFeasible(copied_state);
  ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));

  // ``isStateValid`` が呼ばれると，次の３つのチェックが行われます: (a)
  // 衝突判定 (b) 拘束条件の確認 そして (c) ユーザの設定したコールバックでの実現可能性

  bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
  ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));

  // 現在MoveItとOMPLで利用できる全てのプランナで，衝突判定と拘束条件，
  // ユーザの設定したコールバックでの実現可能性が行えます．
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
