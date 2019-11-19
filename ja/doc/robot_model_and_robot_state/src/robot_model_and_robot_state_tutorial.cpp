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

/* Author: Sachin Chitta, Michael Lautman*/

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // まず
  // ^^^^^^^^^^^^
  // 始めるにあたって :moveit_core:`RobotModel` クラスを使ってセットアップすると簡単です．
  // 一般的に，
  // :moveit_core:`RobotModel` クラスにsharedポインタを返すものが最も難しいです．
  // 可能であるならばいつもそちらを使ったほうがいいです．今回の例では上記のようなsharedポインタから始めて，
  // 基本的なAPIについてのみ触れます．
  // 実際のAPIのコードのクラスを見ることで，これらのクラスによって与えられる特性の利用方法についての情報を得られます． 
  //
  // ROSパラメータサーバでロボットの状態を調べ，
  // :moveit_core:`RobotModel` クラスを構築する
  // `RobotModelLoader`_ オブジェクトをまずインスタンス化します．
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // :moveit_core:`RobotModel` を使うと，
  // ロボットのコンフィグレーションを維持してくれる :moveit_core:`RobotState` を構築できます．
  // state内にすべてのデフォルトの関節角を記録します．
  // すると
  // 特定のグループのロボットモデルを表示する，
  // 本チュートリアルではPandaロボットの ``panda_arm`` ， :moveit_core:`JointModelGroup` を得ることが出来ます．
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 関節角を求める
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Panda armの状態に記録されている現在の関節角の情報を受け取ります．
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 限界関節角
  // ^^^^^^^^^^^^^^^^^^^^^^^^
  // ``setJointGroupPositions()`` は関数自身では限界関節角を設定しませんが， ``enforceBounds()`` を呼び出すことで変わりに行います．
  /* Panda armの関節の一つを限界関節角から外してください */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* 他の関節が限界関節角から外れているかどうか確認してください */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* 限界関節角を設定し，確認してください */
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics（順運動学）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // ここではわかっていない関節角を求めるために，forward kinematicsを計算します．
  // ``panda_arm`` の内で最もエンドエフェクタから遠い ``panda_link8`` の姿勢を求めます．
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  /* エンドエフェクタの姿勢を表示する．これはモデルフレームであることは留意しておいてください */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // Inverse Kinematics（逆運動学）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // ここではPandaロボットのinverse kinematics (IK)を求めます．
  // IKを解くには，下記のものが必要となります:
  //
  //  * 任意のエンドエフェクタの姿勢（通常， ``panda_arm`` チェーンの最後のリンク） :
  //    上記のステップで計算された ``end_effector_state`` のこと．
  //  * タイムアウト: 0.1 s
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // （解があるならば）IK解を出力できるようになりました:
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // ヤコビアンを求める
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // :moveit_core:`RobotState` からヤコビアンを取得できます．
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
