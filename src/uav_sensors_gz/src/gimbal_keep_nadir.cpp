#include "uav_sensors_gz/gimbal_keep_nadir.hpp"

#include <gz/plugin/Register.hh>
#include <sdf/Element.hh>
#include <iostream>

using namespace uav_sensors_gz;

void GimbalKeepNadir::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &)
{
  this->modelEntity = entity;

  if (sdf->HasElement("camera_link"))
  {
    this->cameraLinkName = sdf->Get<std::string>("camera_link");
  }

  gz::sim::Model model(this->modelEntity);
  this->cameraLinkEntity = model.LinkByName(ecm, this->cameraLinkName);

  if (this->cameraLinkEntity == gz::sim::kNullEntity)
  {
    std::cerr << "[GimbalKeepNadir] Could not find link: "
              << this->cameraLinkName << std::endl;
  }
}

void GimbalKeepNadir::PostUpdate(
    const gz::sim::UpdateInfo &,
    const gz::sim::EntityComponentManager &ecm)
{
  if (this->cameraLinkEntity == gz::sim::kNullEntity)
    return;

  // Nadir: pitch = -pi/2
  gz::math::Pose3d target(0, 0, 0, 0, -M_PI_2, 0);

  auto poseComp =
      ecm.Component<gz::sim::components::Pose>(this->cameraLinkEntity);
  if (poseComp)
  {
    gz::math::Pose3d current = poseComp->Data();
    target.Pos() = current.Pos();
    const_cast<gz::sim::components::Pose*>(poseComp)->Data() = target;
  }
}

GZ_ADD_PLUGIN(
  uav_sensors_gz::GimbalKeepNadir,
  gz::sim::System,
  uav_sensors_gz::GimbalKeepNadir::ISystemConfigure,
  uav_sensors_gz::GimbalKeepNadir::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  uav_sensors_gz::GimbalKeepNadir,
  "uav_sensors_gz::GimbalKeepNadir")
