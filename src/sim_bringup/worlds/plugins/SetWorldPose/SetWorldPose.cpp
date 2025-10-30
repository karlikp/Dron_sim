#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/plugin/Register.hh>
#include <ignition/math/Pose3.hh>

using namespace gz;
using namespace sim;

class SetPosePlugin : public System,
                      public ISystemConfigure,
                      public ISystemPostUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->model = Model(_entity);
    this->newPose = ignition::math::Pose3d(2, 0, 1, 0, 0, 1.57);  // x, y, z, roll, pitch, yaw
    this->setPose = true;
  }

  void PostUpdate(const UpdateInfo &, EntityComponentManager &_ecm) override
  {
    if (this->setPose)
    {
      _ecm.SetComponentData(components::Pose(this->model.Entity()), this->newPose);
      this->setPose = false;
    }
  }

private:
  Model model{kNullEntity};
  ignition::math::Pose3d newPose;
  bool setPose{false};
};

GZ_ADD_PLUGIN(SetPosePlugin,
              sim::System,
              ISystemConfigure,
              ISystemPostUpdate)
