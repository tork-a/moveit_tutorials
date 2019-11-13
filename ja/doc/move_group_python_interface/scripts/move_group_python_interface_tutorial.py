#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## Python MoveIt インターフェースを使うため， `moveit_commander`_ namespaceを導入します．
## このnamespaceは `MoveGroupCommander`_ と `PlanningSceneInterface`_ ， `RobotCommander`_ のクラスを提供してくれます．また， `rospy`_ といくつかのメッセージ型をインポートします:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## まず `moveit_commander`_ と `rospy`_ ノードを初期化します:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## オブジェクト `RobotCommander`_ を生成します． ロボットの
    ## 運動学モデルと現在の関節の情報を提供してくれます
    robot = moveit_commander.RobotCommander()

    ## オブジェクト `PlanningSceneInterface`_ を生成します．これはロボットを取り巻く環境を理解するために必要な情報を得る・設定する・変更するのに必要な遠隔インタフェースを提供してくれます:
    scene = moveit_commander.PlanningSceneInterface()

    ## `MoveGroupCommander`_ オブジェクトを初期化します．このオブジェクトはplanning group（関節グループ）のためのインタフェースです．
    ## このチュートリアルではplanning groupはPandaの一番はじめの腕の関節であるので，
    ## planning groupの名前は "panda_arm" としてあります．
    ## もし他のロボットを利用する際には，今回変更してあるplanning groupの名前をそのロボットの名前に変更してください．
    ## このインタフェースは計画と実行をするのに用いられます:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Rvizで軌道を表示するための
    ## `DisplayTrajectory`_ ROS publisher を作ります:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## 基本情報の取得
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # ロボットの参照する座標系の名前を取得します:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # このグループのエンドエフェクタのリンクの名前を表示します:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # ロボットのすべてのグループのリストを取得します:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # ロボットの現在状態を表示しておくとて，デバッグがしやすくなります．
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## 目標関節角度への計画
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Pandaのゼロ・コンフィギュレーションは， `特異姿勢 <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ になった際に起こります．
    ## なので，まずはこの姿勢を回避するようにPandaを動かします．
    # 関節角をグループから取得し，それらのいくつかを調整します:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # goコマンドは関節角や姿勢を引数に取って実行するか，予めグループの姿勢や関節のパラメータを設定しておけば，
    # 引数なしで実行することもできます．
    move_group.go(joint_goal, wait=True)

    # stop() を呼び出すことで，残りの動作がないことを保証します
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## 目標姿勢への計画
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## エンドエフェクタの目標姿勢に向かって，設定しているグループの
    ## 動作計画を行うことができます:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## ここでは動作計画を行うためのプランナを呼び出し，実行します．
    plan = move_group.go(wait=True)
    # stop() を呼び出すことで，残りの動作がないことを保証します．
    move_group.stop()
    # 姿勢を計画したら毎回ターゲットを消すようにしましょう.
    # Note: clear_joint_value_targets() と同じような関数は他にありません
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## 直動軌道
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## エンドエフェクタの経路座標を明示することで直接直動軌道を計画することができます．
    ## もし，Python シェルを使って対話的に実行する場合には, ``scale = 1.0`` と設定してください．
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # はじめにはｚ方向に振り上げ
    wpose.position.y += scale * 0.2  # そしてｙ軸と水平に動く
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # 次にｘ軸に沿って前後に動く
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # 最後にｙ軸と水平に動く
    waypoints.append(copy.deepcopy(wpose))

    # 直動動作を1cmに区切りたいため，直動動作への変換最大ステップ（ `eef_step` ）を0.01に設定しています．
    # `jump_threshold` を0.0に設定しているのは，各ステップを飛ばさないように設定するためです．
    # **警告：ジャンプ閾値を無効にすると，冗長関節を持つロボットの場合は予測不可能な動作を起こし，
    # 危険な動作が起きる可能性があるので十分注意してください．
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: 今の段階ではロボットの動作を計画しただけであって，実際に動かしたわけではない:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## 経路の表示
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## RVizに計画 (aka trajectory) を可視化させることができます．しかし
    ## group.plan() method は自動的にこの作業を行ってくれるので，ここではあまり重要なことではありませんが
    ## 頭の片隅に留めておいてください（下記のコードは再度同じ経路を示しているだけです）:
    ##
    ## `DisplayTrajectory`_ メッセージには trajectory_start と trajectory というメンバがあります．
    ## trajectory_start には通常，AttachedCollisionObjects (ロボットに取り付けた衝突物体)の情報を含む現在の
    ## ロボットの状態をコピーし，trajectory には生成したパス(軌道)をコピーします．
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## 計画の実行
    ## ^^^^^^^^^^^^^^^^
    ## ロボットに計画した経路を実行させたければ， ``execute()`` 関数を使用してくださいい:
    move_group.execute(plan, wait=True)

    ## **Note:** ロボットの現在の関節の状態が
    ## `RobotTrajectory`_ に設定されている公差の中に収まっていない場合， ``execute()`` を実行しても失敗します．
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## 衝突の更新がされているか確かめる
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 障害物メッセージの更新を実行する前にPythonノードが失敗したならば，
    ## メッセージは失われ立方体は現れません．更新されたことを確かめるには
    ## ``get_attached_objects()`` と ``get_known_object_names()`` のリストに変更があるまで待ちます．
    ## 本チュートリアルでは，
    ## プラニング環境に物体を追加，除去，取り付け/取り外しした後にこの関数を呼びます．
    ## そして，物体の情報が更新されるのを， ``timeout`` に設定した時間までは待ちます．
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # 立方体が物体と触れているか確かめます
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # 立方体が環境にあるか確かめて行きます．
      # ロボットに立方体を取り付けるとknown_objectsから削除されていることを確認してください．
      is_known = box_name in scene.get_known_object_names()

      # ここまでの手順をしっかり踏んでいるか確認します
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # 他のスレッドを更新するために待機します
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # 返り値を返さずにループを抜けたら，タイムアウトなのでFalseを返します．
    return False
    ## END_SUB_TUTORIAL


  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## プランニング環境に物体を追加する
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## まず，プランニング環境内のロボットの左指の位置に箱を作ります:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # エンドエフェクタから少し離す
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## ロボットに物体を取り付ける
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 次に，Pandaの手首に立方体を取り付けます．物体の操作を行うには
    ## プランニング環境内で操作したい物体とロボットが衝突していると判定されないようにする必要があります．
    ## ``touch_links`` 配列にリンク名を追加することで,
    ## プランニング環境に追加されたリンクと物体間での衝突判定を無視するように設定します．
    ## Pandaロボットでは， ``grasping_group = 'hand'`` と設定しています．もし，他のロボットを使う際には，
    ## そのロボットで設定しているエンドエフェクタのグループ名を設定してください．
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## ロボットから物体を取り外す
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## プランニング環境内のロボットから物体を取り外し，消すことも可能です:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## 物体をプランニング環境から消す
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 環境から物体を削除することができます．
    scene.remove_world_object(box_name)

    ## **Note:** 物体を消す前にロボットから取り外さなければなりません．    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    tutorial.add_box()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    tutorial.attach_box()

    print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    tutorial.detach_box()

    print "============ Press `Enter` to remove the box from the planning scene ..."
    raw_input()
    tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
