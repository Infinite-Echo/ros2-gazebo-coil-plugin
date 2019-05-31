#include <wibotic_gazebo_plugins/WiboticCoilPlugin.h>
#include <msgs/coil_position.pb.h>
#include <vector>
#include <cmath>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(WiboticCoilPlugin)

typedef wibotic_gazebo_plugins_msgs::msgs::CoilPositionRequest CoilPositionRequest;

static std::vector<physics::ModelPtr> receive_coils;
static std::vector<physics::ModelPtr> transmit_coils;

/////////////////////////////////////////////////
WiboticCoilPlugin::WiboticCoilPlugin()
{}

/////////////////////////////////////////////////
WiboticCoilPlugin::~WiboticCoilPlugin()
{}

/////////////////////////////////////////////////
void WiboticCoilPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  model_ptr_ = model;
  
  if (model_ptr_->GetChildLink("coil_center") == nullptr) {
    gzerr << "Plugin requires model have a link coil_center defined.\n";
    return;
  }
  
  if (sdf->HasElement("coilType")) {
    const std::string coil_type = sdf->GetElement("coilType")->Get<std::string>();
    if (!coil_type.compare("receive")) {
      coil_type_ = CoilType::RECEIVE;
      receive_coils.push_back(model_ptr_);
    } else if (!coil_type.compare("transmit")) {
      coil_type_ = CoilType::TRANSMIT;
      transmit_coils.push_back(model_ptr_);
    } else {
      gzerr << "coilType not recognized. Must be either \"receive\" or \"transmit\".\n";
      return;
    }
  } else {
    gzerr << "Plugin requires <coilType> be specified.\n";
    return;
  }
  
  std::string publish_to = "/gazebo/wibotic/coil_position";
  if (sdf->HasElement("topic")) {
    publish_to = sdf->GetElement("topic")->Get<std::string>();
  } else {
    gzwarn << "No <topic> specified, publishing to " << publish_to << "\n";
  }
    
  // Setup Gazebo transport node
  transport::NodePtr node(new transport::Node());
  transport_node_ = node;
  transport_node_->Init();
  transport_pub_ = transport_node_->Advertise<CoilPositionRequest>(publish_to);
    
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&WiboticCoilPlugin::OnUpdate, this)
  );
}

void WiboticCoilPlugin::OnUpdate() {
  if (!transport_pub_->HasConnections()) {
    return;
  }
  
  CoilPositionRequest request;
  request.set_model1(model_ptr_->GetName());
  
  const std::vector<physics::ModelPtr>& matching_coils = 
    (coil_type_ == CoilType::RECEIVE) ? transmit_coils : receive_coils;

  // No matching coil in the world, no sense checking positions
  if (matching_coils.size() < 1) {
    transport_pub_->Publish(request);
    return;
  }
  
  const auto& pose = model_ptr_->GetChildLink("coil_center")->WorldPose();
  const ignition::math::Vector3<double>& my_rotation = pose.Rot().Euler();
  const ignition::math::Vector3<double> my_position = pose.Pos();
  
  // Check position against every other model that has this plugin
  for (const auto model : matching_coils) {
    const auto& other_pose = model->GetChildLink("coil_center")->WorldPose();
    const ignition::math::Vector3<double>& other_rotation = other_pose.Rot().Euler();
    const ignition::math::Vector3<double>& other_position = other_pose.Pos();
    
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
    
    request.set_model2(model->GetName());
    request.set_optimality(optimality);
    request.set_angle_optimality(angle_norm);
    request.set_position_optimality(distance_norm);
    
    transport_pub_->Publish(request);
  }
}