#include "DoorPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DoorPlugin)


/////////////////////////////////////////////////
DoorPlugin::DoorPlugin()
{

}

/////////////////////////////////////////////////
void DoorPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->world = _parent;

  // Create a new transport node
  this->node.reset(new transport::Node());

  // Initialize the node with the world name
  this->node->Init(this->world->GetName());

  this->taskboard = this->world->GetModel("taskboard_demo");
  this->door =  this->world->GetModel("iss_door_A");

  this->addTorque = false;
  this->drill =  this->world->GetModel("drill");

  this->prevTime = common::Time::Zero;

  this->doorOpen = false;
  this->toOpen = false;

  this->switchJoint =
      this->taskboard->GetJoint("taskboard_lower/taskboard_slot1_switch3");

  this->doorInitialPose = this->door->GetWorldPose();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&DoorPlugin::Update, this, _1));

  this->worldControlSub = this->node->Subscribe("~/world_control",
      &DoorPlugin::OnWorldControl, this);

  this->hydraSub = this->node->Subscribe("~/hydra",
      &DoorPlugin::OnHydra, this);
}

/////////////////////////////////////////////////
void DoorPlugin::OnHydra(ConstHydraPtr &_msg)
{
  if (_msg->right().button_4() &&_msg->left().button_3())
  {
    gazebo::common::Time now = gazebo::common::Time::GetWallTime();
    if ((now - this->prevTime) < common::Time(1, 0))
      return;

    this->prevTime = now;

    this->toOpen = !this->doorOpen;

    return;
  }
}

/////////////////////////////////////////////////
void DoorPlugin::OpenDoor(bool _open)
{
  if (!this->door)
    return;

  if (!_open)
  {
    // set it back to initial pose.
    this->door->SetWorldPose(this->doorInitialPose);
  }
  else
  {
    this->door->SetWorldPose(math::Pose(2.64, 2.18, 1.21, 0, -1.57, 0));
  }

}

/////////////////////////////////////////////////
void DoorPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  if (this->switchJoint)
  {
    if (this->switchJoint->GetAngle(0).Radian() < -0.5)
    {
      this->toOpen = true;
      this->OpenDoor(true);
    }
    else
    {
    }
  }

  if (this->door->GetWorldPose() == this->doorInitialPose)
  {
    this->doorOpen = false;
  }
  else
  {
    this->doorOpen = true;
  }

  //if (this->toOpen != this->doorOpen)
  {
    this->OpenDoor(this->toOpen);
  }

  if (this->drill && !this->addTorque)
  {
    this->drill->GetLink()->SetAngularVel(math::Vector3(0.08, 0, 0));
    this->drill->GetLink()->SetLinearVel(math::Vector3(-0.006, 0.001, 0));
    this->addTorque = true;
  }

}

/////////////////////////////////////////////////
void DoorPlugin::OnWorldControl(ConstWorldControlPtr &_msg)
{
  if (_msg->has_reset())
  {
    // set it back to initial pose.
    this->door->SetWorldPose(this->doorInitialPose);

    this->drill->GetLink()->SetAngularVel(math::Vector3(0, 0, 0));
    this->drill->GetLink()->SetLinearVel(math::Vector3(0.0, 0.0, 0));
    this->addTorque = false;
    this->toOpen = false;
    this->doorOpen = false;
  }
}

