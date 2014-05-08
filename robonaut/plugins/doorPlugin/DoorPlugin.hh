#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

/// \example examples/plugins/world_edit.cc
/// This example creates a WorldPlugin, initializes the Transport system by
/// creating a new Node, and publishes messages to alter gravity.
namespace gazebo
{
  class DoorPlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: DoorPlugin();

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private: void Update(const common::UpdateInfo &_info);

    private: void OnWorldControl(ConstWorldControlPtr &_msg);

    private: void OnHydra(ConstHydraPtr &_msg);

    private: void OpenDoor(bool _open);

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::WorldPtr world;

    private: physics::ModelPtr door;

    private: physics::ModelPtr taskboard;

    private: physics::ModelPtr drill;

    private: physics::JointPtr switchJoint;

    private: math::Pose doorInitialPose;

    private: transport::NodePtr node;

    private: transport::SubscriberPtr worldControlSub;

    private: transport::SubscriberPtr hydraSub;

    private: common::Time prevTime;

    private: bool doorOpen;

    private: bool toOpen;

    private: bool addTorque;
  };
}
