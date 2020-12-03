#include <wibotic_gazebo_plugins/WiboticCoilPlugin.h>
#include <wibotic_gazebo_plugins/VersionShim.h>
#include <wibotic_gazebo_plugins/CoilPosition.h>
#include <vector>
#include <cmath>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(WiboticCoilPlugin)

static std::vector<physics::CollisionPtr> receive_coils;
static std::vector<physics::CollisionPtr> transmit_coils;

const physics::CollisionPtr find_center(const physics::ModelPtr model) {
  const physics::Link_V& model_links = model->GetLinks();
  for (const auto& link : model_links) {
    const physics::Collision_V link_collisions = link->GetCollisions();
    for (const auto& collision : link_collisions) {
      const std::string collision_name = collision->GetName();
      if (collision_name.find("COIL_CENTER") != std::string::npos) {
        return collision;
      }
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
WiboticCoilPlugin::WiboticCoilPlugin()
{}

/////////////////////////////////////////////////
WiboticCoilPlugin::~WiboticCoilPlugin()
{}

/////////////////////////////////////////////////
void WiboticCoilPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ptr_ = model;
  coil_center_ = find_center(model_ptr_);
  
  if (coil_center_ == nullptr) {
    gzerr << "Plugin requires model have a collision COIL_CENTER defined.\n";
    return;
  }
  
  if (sdf->HasElement("coilType")) {
    const std::string coil_type = sdf->GetElement("coilType")->Get<std::string>();
    if (!coil_type.compare("receive")) {
      coil_type_ = CoilType::RECEIVE;
      receive_coils.push_back(coil_center_);
    } else if (!coil_type.compare("transmit")) {
      coil_type_ = CoilType::TRANSMIT;
      transmit_coils.push_back(coil_center_);
    } else {
      gzerr << "coilType not recognized. Must be either \"receive\" or \"transmit\".\n";
      return;
    }
  } else {
    gzerr << "Plugin requires <coilType> be specified.\n";
    return;
  }
  
  std::string publish_to = "/wibotic/coil_position";
  if (sdf->HasElement("topic")) {
    publish_to = sdf->GetElement("topic")->Get<std::string>();
  } else {
    gzwarn << "No <topic> specified, publishing to " << publish_to << "\n";
  }
  
  pub_ = nh_.advertise<wibotic_gazebo_plugins::CoilPosition>(publish_to, 10);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&WiboticCoilPlugin::OnUpdate, this)
  );
}

void WiboticCoilPlugin::OnUpdate() {
  wibotic_gazebo_plugins::CoilPosition ros_msg;
  
  // Default ros message
  ros_msg.model_1 = model_ptr_->GetName();
  ros_msg.optimality = 0;
  
  const std::vector<physics::CollisionPtr>& matching_coils = 
    (coil_type_ == CoilType::RECEIVE) ? transmit_coils : receive_coils;

  // No matching coil in the world, no sense checking positions
  if (matching_coils.size() < 1) {
    pub_.publish(ros_msg);
    return;
  }
  
  const auto& pose = coil_center_->WorldPose();
  const ignition::math::Vector3<double>& my_rotation = pose.GetRotation();
  const ignition::math::Vector3<double>& my_position = pose.GetPosition();
  
  // Check position against every other model that has this plugin
  for (const auto& collision : matching_coils) {
    const auto& other_pose = collision->WorldPose();
    const ignition::math::Vector3<double>& other_rotation = other_pose.GetRotation();
    const ignition::math::Vector3<double>& other_position = other_pose.GetPosition();
    
    ignition::math::Vector3<double> angles = my_rotation - other_rotation;
    if (angles.X() > PI) {
      angles.X() -= TWO_PI;
    } else if (angles.X() < -PI) {
      angles.X() += TWO_PI;
    }
    if (angles.Y() > PI) {
      angles.Y() -= TWO_PI;
    } else if (angles.Y() < -PI) {
      angles.Y() += TWO_PI;
    }
    if (angles.Z() > PI) {
      angles.Z() -= TWO_PI;
    } else if (angles.Z() < -PI) {
      angles.Z() += TWO_PI;
    }
    
    // Optimal angle is when angles are identical for both coils
    // If a coil is rotated past +/- pi/4 relative to the other, it is fully unoptimal
    const double angle_norm = std::max(0.0,
      std::cos(2 * std::min(std::abs(angles.X()), PI_OVER_4)) * 
      std::cos(2 * std::min(std::abs(angles.Y()), PI_OVER_4)) * 
      std::cos(2 * std::min(std::abs(angles.Z()), PI_OVER_4))
    );
    
    // Calculate the 3D point-to-point distance for the two coils
    const ignition::math::Vector3<double>& positions = my_position - other_position;
    const double distance = std::sqrt(
      std::pow(positions.X(), 2) + 
      std::pow(positions.Y(), 2) + 
      std::pow(positions.Z(), 2)
    );
    // Bound the distance between 0 and MAX_RANGE and normalize
    // Optimal distance will be at the MAX_RANGE / 2 point
    const double distance_norm = std::max(0.0, 
      std::sin((PI * (std::min(MAX_RANGE, distance) / MAX_RANGE)))
    );
    
    const double optimality = angle_norm * distance_norm;
    
    // std::stringstream ss;
    // ss << model_ptr_->GetName() << "\n"
    //   << collision->GetParentModel()->GetName() << "\n"
    //   << "Angles:\tX: " << angles.X() << "\tY: " << angles.Y() << "\tZ: " << angles.Z() << "\n"
    //   << "Angle Norm: " << angle_norm << "\n"
    //   << "My Position:\tX:" << my_position.X() << "\tY: " << my_position.Y() << "\tZ: " << my_position.Z() << "\n"
    //   << "Other Position:\tX:" << other_position.X() << "\tY: " << other_position.Y() << "\tZ: " << other_position.Z() << "\n"
    //   << "Distance Apart:\tX: " << positions.X() << "\tY: " << positions.Y() << "\tZ: " << positions.Z() << "\n"
    //   << "Distance: " << distance << "\n"
    //   << "Distance Norm: " << distance_norm << "\n"
    //   << "Normed : " << (std::min(MAX_RANGE, distance) / MAX_RANGE) << "\n"
    //   << "Sin'd: " << std::sin((PI * (std::min(MAX_RANGE, distance) / MAX_RANGE))) << "\n"
    //   << "Optimality: " << optimality << "\n";
    // std::cout << ss.str() << std::endl;
    
    ros_msg.model_2 = collision->GetParentModel()->GetName();
    ros_msg.optimality = optimality;
    ros_msg.angle_optimality = angle_norm;
    ros_msg.position_optimality = distance_norm;
  
    pub_.publish(ros_msg);
  }
}
