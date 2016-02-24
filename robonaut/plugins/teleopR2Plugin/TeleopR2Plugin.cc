/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/transport/transport.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "TeleopR2Plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TeleopR2Plugin)

/////////////////////////////////////////////////
TeleopR2Plugin::TeleopR2Plugin()
{
  this->activated = false;
  this->yaw = 0;
}

/////////////////////////////////////////////////
TeleopR2Plugin::~TeleopR2Plugin()
{
}

/////////////////////////////////////////////////
void TeleopR2Plugin::OnHydra(ConstHydraPtr &_msg)
{
  if (_msg->right().button_center() &&_msg->left().button_center())
  {
    this->Restart();
    return;
  }

  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void TeleopR2Plugin::Restart()
{
  this->rightModel->SetRelativePose(this->rightStartPose);
  this->leftModel->SetRelativePose(this->leftStartPose);
  this->basePoseRight = this->rightStartPose;
  this->basePoseLeft = this->leftStartPose;
  this->rightBumper = false;
  this->leftBumper = false;
  this->activated = false;
  this->yaw = 0;
  this->prevModelPose = this->modelStartPose;
  this->pelvisTarget = this->pelvisStartPose;

  this->dollyPinJoint->Detach();
  this->dolly->SetWorldPose(this->dollyStartPose);
  this->dollyPinJoint->Attach(physics::LinkPtr(), this->dolly->GetLink("link"));

  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);
  this->worldControlPub->Publish(msg);

  //this->world->Reset();
}

/////////////////////////////////////////////////
void TeleopR2Plugin::SetRightFingers(double _angle)
{
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/index/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/index/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/middle/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/middle/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/little/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/little/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/little/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/ring/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/ring/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/right_arm/hand/ring/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/right_arm/hand/thumb/joint3", _angle);
}

/////////////////////////////////////////////////
void TeleopR2Plugin::SetLeftFingers(double _angle)
{
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/index/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/index/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/middle/joint2", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/middle/joint3", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/little/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/little/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/little/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/ring/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/ring/joint1", _angle);
  //this->jointController->SetPositionTarget(
  //    "r2::/r2/left_arm/hand/ring/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint0", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint1", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint2", _angle);
  this->jointController->SetPositionTarget(
      "r2::/r2/left_arm/hand/thumb/joint3", _angle);
}

/////////////////////////////////////////////////
void TeleopR2Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  this->rightModel = this->model->GetLink("right_arm_goal_link");
  this->leftModel = this->model->GetLink("left_arm_goal_link");

  if (!this->rightModel)
    gzerr << "Unable to get right arm goal link\n";

  if (!this->leftModel)
    gzerr << "Unable to get left arm goal link\n";

  ignition::math::Quaterniond modelRot =
    this->model->GetWorldPose().Ign().Rot();

  this->basePoseRight = this->rightModel->GetRelativePose().Ign();
  this->basePoseLeft = this->leftModel->GetRelativePose().Ign();

  this->rightStartPose = this->basePoseRight;
  this->leftStartPose = this->basePoseLeft;
  this->modelStartPose = this->model->GetWorldPose().Ign();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->hydraSub = this->node->Subscribe("~/hydra",
      &TeleopR2Plugin::OnHydra, this);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");

  this->jointController = this->model->GetJointController();

  this->pinJoint = this->world->GetPhysicsEngine()->CreateJoint(
      "revolute", this->model);

  if (_sdf->HasElement("pin_link"))
  {
    std::string pinModelStr =_sdf->Get<std::string>("pin_model");
    std::string pinLinkStr =_sdf->Get<std::string>("pin_link");

    this->dolly = this->world->GetModel(pinModelStr);
    this->dollyStartPose = this->dolly->GetWorldPose().Ign();

    if (!this->dolly)
      gzerr << "Unable to get pin model[" << pinModelStr << "]\n";
    else
    {
      this->pinLink = this->dolly->GetLink(pinLinkStr);

      if (!this->pinLink)
        gzerr << "Unable to get pin link[" << pinLinkStr << "]\n";
    }

    this->dollyPinJoint = this->dolly->GetJoint("world_joint");
  }

  this->pinJoint->SetModel(this->model);

  this->pinJoint->Load(this->pinLink, this->model->GetLink("base"),
      ignition::math::Pose3d());
  this->pinJoint->SetUpperLimit(0,0);
  this->pinJoint->SetLowerLimit(0,0);
  this->pinJoint->Init();

  std::vector<std::string> rightHandJoints;
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/index/joint3");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/middle/joint3");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/little/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/ring/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint0");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint1");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint2");
  rightHandJoints.push_back("/r2/right_arm/hand/thumb/joint3");

  std::vector<std::string> leftHandJoints;
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/index/joint3");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/middle/joint3");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/little/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/ring/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint0");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint1");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint2");
  leftHandJoints.push_back("/r2/left_arm/hand/thumb/joint3");

  for (std::vector<std::string>::iterator iter = rightHandJoints.begin();
      iter != rightHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("r2::" + *iter,
        common::PID(3.0, 0.0, 0.1));

    this->jointController->SetPositionTarget("r2::" + *iter, 0);
  }

  for (std::vector<std::string>::iterator iter = leftHandJoints.begin();
      iter != leftHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("r2::" + *iter,
        common::PID(3.0, 0.0, 0.1));

    this->jointController->SetPositionTarget("r2::" + *iter, 0);
  }

  this->jointController->SetPositionPID(
      "r2::/r2/right_arm/hand/thumb/joint1",
      common::PID(0.5, 0.0, 0.1));
  this->jointController->SetPositionPID(
      "r2::/r2/left_arm/hand/thumb/joint1",
      common::PID(0.5, 0.0, 0.1));
   this->jointController->SetPositionPID(
      "r2::/r2/right_arm/hand/thumb/joint2",
      common::PID(0.5, 0.0, 0.1));
  this->jointController->SetPositionPID(
      "r2::/r2/left_arm/hand/thumb/joint2",
      common::PID(0.5, 0.0, 0.1));

  this->rightBumper = false;
  this->leftBumper = false;

  gzerr << "Robonaut Plugin Loaded!" << std::endl;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TeleopR2Plugin::Update, this, _1));
}

/////////////////////////////////////////////////
void TeleopR2Plugin::Update(const common::UpdateInfo & /*_info*/)
{
  boost::mutex::scoped_lock lock(this->msgMutex);

  if (this->hydraMsgs.empty())
    return;

  if (!this->activated)
  {
    for (std::list<boost::shared_ptr<msgs::Hydra const> >::iterator iter =
        this->hydraMsgs.begin(); iter != this->hydraMsgs.end(); ++iter)
    {
      if ((*iter)->right().button_bumper() && (*iter)->left().button_bumper())
      {
        this->activated = true;
        this->resetPoseRight = msgs::ConvertIgn((*iter)->right().pose());
        this->resetPoseLeft = msgs::ConvertIgn((*iter)->left().pose());
        break;
      }
    }
  }

  if (this->activated)
  {
    boost::shared_ptr<msgs::Hydra const> msg = this->hydraMsgs.back();

    ignition::math::Pose3d rightPose;
    ignition::math::Pose3d leftPose;

    ignition::math::Pose3d rightAdjust, leftAdjust;

    rightPose = msgs::ConvertIgn(msg->right().pose());
    leftPose = msgs::ConvertIgn(msg->left().pose());



    rightAdjust = ignition::math::Pose3d(
      this->modelStartPose.Inverse().Rot() *
        (rightPose.Pos() - this->resetPoseRight.Pos()) +
        this->basePoseRight.Pos(),
        this->modelStartPose.Inverse().Rot() *
        rightPose.Rot() * this->resetPoseRight.Rot().Inverse() *
        this->basePoseRight.Rot());

   /*std::cerr << " ----- " << std::endl;
   std::cerr << " right pose " << rightPose.pos << std::endl;
   std::cerr << " reset pose right " << resetPoseRight.pos << std::endl;
   std::cerr << " right - reset " << rightPose.pos - this->resetPoseRight.pos << std::endl;
   std::cerr << " base pose right " << basePoseRight.pos << std::endl;
   std::cerr << " model rot " << this->model->GetRelativePose().rot.GetAsEuler() << std::endl;
   std::cerr << " right adjust " << rightAdjust.pos << std::endl;*/



   //gzerr << " right adjust " << rightAdjust.pos << " vs " << this->model->GetRelativePose().rot*rightAdjust.pos << std::endl;

   // gzerr << " right adjust " << this->model->GetRelativePose().rot.GetAsEuler() << std::endl;

    leftAdjust = ignition::math::Pose3d(this->modelStartPose.Inverse().Rot() *
        (leftPose.Pos() - this->resetPoseLeft.Pos()) +
        this->basePoseLeft.Pos(),
        this->modelStartPose.Inverse().Rot() *
        leftPose.Rot() * this->resetPoseLeft.Rot().Inverse() *
        this->basePoseLeft.Rot());

    //rightAdjust.pos = this->model->GetRelativePose().rot*rightAdjust.pos;
    //leftAdjust.pos = this->model->GetRelativePose().rot*leftAdjust.pos;

    this->SetRightFingers(msg->right().trigger()*1.5707);
    this->SetLeftFingers(msg->left().trigger()*1.5707);

    common::Time curTime = common::Time::GetWallTime();
    double dt = (curTime - this->prevTime).Double();

    double rx = msg->right().joy_x() * 0.5 * dt;
    double ry = msg->right().joy_y() * -0.5 * dt;

    this->yaw += msg->left().joy_y() * -.5 * dt;
    double dx = rx * cos(this->yaw) + ry*sin(this->yaw*-1);
    double dy = rx * sin(this->yaw) + ry*cos(this->yaw*-1);
    ignition::math::Pose3d dPose(
      dx, dy, 0, 0, 0, msg->left().joy_y() * -.5 * dt);

    ignition::math::Vector3d rpy =
      this->dolly->GetWorldPose().Ign().Rot().Euler();
    rpy.Z(yaw);
    rpy.X(this->dollyStartPose.Rot().X());
    rpy.Y(this->dollyStartPose.Rot().Y());
    //std::cerr << " rpy x y " << rpy.x << " " << rpy.y << std::endl;

    this->dollyPinJoint->Detach();
    ignition::math::Vector3d dollyPos =
      this->dolly->GetWorldPose().Ign().Pos() + dPose.Pos();
    dollyPos.Z(this->dollyStartPose.Pos().Z());
    this->dolly->SetWorldPose(
      ignition::math::Pose3d(dollyPos, ignition::math::Quaterniond(rpy)));

    this->dollyPinJoint->Attach(physics::LinkPtr(),
        this->dolly->GetLink("link"));

    this->rightModel->SetRelativePose(rightAdjust);
    this->leftModel->SetRelativePose(leftAdjust);

    this->rightBumper = msg->right().button_bumper();
    this->leftBumper = msg->left().button_bumper();

    this->prevTime = curTime;
  }

  this->hydraMsgs.clear();
}
