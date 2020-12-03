#ifndef WIBOTIC_COIL_PLUGIN_HH_
#define WIBOTIC_COIL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>


using namespace gazebo;

class WiboticCoilPlugin : public ModelPlugin {
  public:
    WiboticCoilPlugin();
    virtual ~WiboticCoilPlugin();
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void OnUpdate();

  private:
    const double MAX_RANGE = 0.08; 
    const double PI = 3.14159265359;
    const double PI_OVER_4 = PI / 4;
    const double TWO_PI = PI * 2;
    
    enum class CoilType : uint8_t {
      TRANSMIT = 0,
      RECEIVE  = 1
    };
    
    physics::ModelPtr model_ptr_;
    physics::CollisionPtr coil_center_;
    
    event::ConnectionPtr update_connection_;
    
    CoilType coil_type_;
    
    ros::NodeHandle nh_;
    ros::Publisher pub_; 
};

#endif // WIBOTIC_COIL_PLUGIN_HH_