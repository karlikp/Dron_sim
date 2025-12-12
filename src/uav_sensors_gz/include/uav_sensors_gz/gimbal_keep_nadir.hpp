#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Pose3.hh>

namespace uav_sensors_gz
{
class GimbalKeepNadir
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate
{
public:
  GimbalKeepNadir() = default;

  void Configure(
      const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &eventMgr) override;

  void PostUpdate(
      const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override;

private:
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity cameraLinkEntity{gz::sim::kNullEntity};
  std::string cameraLinkName{"camera_link"};
};
} 
