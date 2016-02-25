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
#include "TeleopAtlasPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TeleopAtlasPlugin)

/////////////////////////////////////////////////
TeleopAtlasPlugin::TeleopAtlasPlugin()
{
  this->activated = false;
  this->yaw = 0;
}

/////////////////////////////////////////////////
TeleopAtlasPlugin::~TeleopAtlasPlugin()
{
}

/////////////////////////////////////////////////
void TeleopAtlasPlugin::OnHydra(ConstHydraPtr &_msg)
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
void TeleopAtlasPlugin::Restart()
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
void TeleopAtlasPlugin::SetRightFingers(double _angle)
{
  this->jointController->SetPositionTarget("atlas::right_f0_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f0_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f1_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f1_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f2_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f2_j2", _angle);

  this->jointController->SetPositionTarget("atlas::right_f3_j1", _angle);
  this->jointController->SetPositionTarget("atlas::right_f3_j2", _angle);
}

/////////////////////////////////////////////////
void TeleopAtlasPlugin::SetLeftFingers(double _angle)
{
  this->jointController->SetPositionTarget("atlas::left_f0_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f0_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f1_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f1_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f2_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f2_j2", _angle);

  this->jointController->SetPositionTarget("atlas::left_f3_j1", _angle);
  this->jointController->SetPositionTarget("atlas::left_f3_j2", _angle);
}

/////////////////////////////////////////////////
void TeleopAtlasPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
      &TeleopAtlasPlugin::OnHydra, this);

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

  this->pinJoint->Load(this->pinLink, this->model->GetLink("pelvis"),
      ignition::math::Pose3d());
  this->pinJoint->SetUpperLimit(0,0);
  this->pinJoint->SetLowerLimit(0,0);
  this->pinJoint->Init();

  std::vector<std::string> rightHandJoints;
  rightHandJoints.push_back("right_f0_j0");
  rightHandJoints.push_back("right_f0_j1");
  rightHandJoints.push_back("right_f0_j2");
  rightHandJoints.push_back("right_f1_j0");
  rightHandJoints.push_back("right_f1_j1");
  rightHandJoints.push_back("right_f1_j2");
  rightHandJoints.push_back("right_f2_j0");
  rightHandJoints.push_back("right_f2_j1");
  rightHandJoints.push_back("right_f2_j2");
  rightHandJoints.push_back("right_f3_j0");
  rightHandJoints.push_back("right_f3_j1");
  rightHandJoints.push_back("right_f3_j2");

  std::vector<std::string> leftHandJoints;
  leftHandJoints.push_back("left_f0_j0");
  leftHandJoints.push_back("left_f0_j1");
  leftHandJoints.push_back("left_f0_j2");
  leftHandJoints.push_back("left_f1_j0");
  leftHandJoints.push_back("left_f1_j1");
  leftHandJoints.push_back("left_f1_j2");
  leftHandJoints.push_back("left_f2_j0");
  leftHandJoints.push_back("left_f2_j1");
  leftHandJoints.push_back("left_f2_j2");
  leftHandJoints.push_back("left_f3_j0");
  leftHandJoints.push_back("left_f3_j1");
  leftHandJoints.push_back("left_f3_j2");

  for (std::vector<std::string>::iterator iter = rightHandJoints.begin();
      iter != rightHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("atlas::" + *iter,
        common::PID(80.5, 0.1, 2.1));

    this->jointController->SetPositionTarget("atlas::" + *iter, 0);
  }

  for (std::vector<std::string>::iterator iter = leftHandJoints.begin();
      iter != leftHandJoints.end(); ++iter)
  {
    this->jointController->SetPositionPID("atlas::" + *iter,
        common::PID(80.5, 0.1, 2.1));

    this->jointController->SetPositionTarget("atlas::" + *iter, 0);
  }

  this->rightBumper = false;
  this->leftBumper = false;

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TeleopAtlasPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void TeleopAtlasPlugin::Update(const common::UpdateInfo & /*_info*/)
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
      rightPose.Pos() - this->resetPoseRight.Pos() +
        this->basePoseRight.Pos(),
        rightPose.Rot() * this->resetPoseRight.Rot().Inverse() *
        this->basePoseRight.Rot());

    leftAdjust = ignition::math::Pose3d(leftPose.Pos() -
      this->resetPoseLeft.Pos() +
        this->basePoseLeft.Pos(),
        leftPose.Rot() * this->resetPoseLeft.Rot().Inverse() *
        this->basePoseLeft.Rot());

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
