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

#ifndef _GAZEBO_TELEOP_R2_PLUGIN_HH_
#define _GAZEBO_TELEOP_R2_PLUGIN_HH_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  class TeleopR2Plugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TeleopR2Plugin();

    /// \brief Destructor
    public: virtual ~TeleopR2Plugin();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    /// \param[in] _info Update information provided by the server.
    private: void Update(const common::UpdateInfo &_info);

    private: void OnHydra(ConstHydraPtr &_msg);

    private: void SetRightFingers(double _angle);
    private: void SetLeftFingers(double _angle);

    private: void Restart();

    private: physics::WorldPtr world;
    private: physics::ModelPtr model;
    private: physics::ModelPtr boxModel;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr hydraSub;
    private: transport::PublisherPtr worldControlPub;

    private: boost::mutex update_mutex;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr pinJoint;

    private: physics::LinkPtr rightHand;

    private: physics::JointControllerPtr jointController;

    private: ignition::math::Vector3d goalPos;

    private: bool rightBumper, leftBumper;

    private: physics::LinkPtr rightModel, leftModel;

    private: ignition::math::Pose3d rightStartPose;
    private: ignition::math::Pose3d leftStartPose;

    private: ignition::math::Pose3d basePoseRight;
    private: ignition::math::Pose3d basePoseLeft;
    private: ignition::math::Pose3d resetPoseRight;
    private: ignition::math::Pose3d resetPoseLeft;
    private: ignition::math::Pose3d modelStartPose;

    private: bool activated;
    private: physics::LinkPtr pinLink;

    private: physics::ModelPtr dolly;
    private: physics::JointPtr dollyPinJoint;

    private: common::PID xPosPID, yPosPID, zPosPID;
    private: common::PID rollPID, pitchPID, yawPID;
    private: ignition::math::Pose3d pelvisTarget, pelvisStartPose;
    private: ignition::math::Pose3d prevModelPose;

    private: ignition::math::Pose3d dollyStartPose;
    private: double yaw;
    private: common::Time prevTime;

    private: boost::mutex msgMutex;
    private: std::list<boost::shared_ptr<msgs::Hydra const> > hydraMsgs;
  };
}
#endif
