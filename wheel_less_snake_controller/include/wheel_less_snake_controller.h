#ifndef WHEEL_LESS_SNAKE_CONTROLLER_H
#define WHEEL_LESS_SNAKE_CONTROLLER_H

#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <wheel_less_snake_controller/SetPIDParameters.h>

#include <ros/ros.h>

#include <algorithm>
#include <mutex>

namespace gazebo
{

#define pi 3.14159

typedef std::map<std::string, physics::JointPtr> JointMap;

struct SecOrdState
{
  float v1;
  float v2;
};

using namespace wheel_less_snake_controller;

class SnakeCPGController : public ModelPlugin
{

public:

  SnakeCPGController();
  ~SnakeCPGController();

  // Load the plugin and initialize all controllers
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Simulation update callback function
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

  // check if controller for current joint is specified in SDF and return pointer to sdf element
  bool existsControllerSDF(sdf::ElementPtr &sdf_ctrl_def, const sdf::ElementPtr &sdf,
                           const physics::JointPtr &joint);

  // get PID controller values from SDF
  common::PID getControllerPID(const sdf::ElementPtr &sdf_ctrl_def);

  // Method for creating a position controller for a given joint
  void createPositionController(const physics::JointPtr &joint, const common::PID &pid_param);

  // Generic position command callback function (ROS topic)
  void radiusCB(const std_msgs::Float64::ConstPtr &msg);
  void ampCB(const std_msgs::Float64::ConstPtr &msg);
  void omegaCB(const std_msgs::Float64::ConstPtr &msg);
  void thetaCB(const std_msgs::Float64::ConstPtr &msg);
  void linRedCB(const std_msgs::Float64::ConstPtr &msg);

  // Generic PID parameter setter callback function (ROS service)
  bool setPIDParametersCB(SetPIDParameters::Request &req,
                          SetPIDParameters::Response &res);

  // ROS node handle
  ros::NodeHandle m_nh;

  // Pointer to the model
  physics::ModelPtr m_model;

  // Pointer to joint controllers
  physics::JointControllerPtr m_joint_controller;
  
  // Map of joint pointers
  JointMap m_joint_map;
  
  // Vector of joint pointers used
  std::vector<physics::JointPtr> m_joints;

  // Pointer to the update event connection
  event::ConnectionPtr m_updateConnection;

  // ROS subscriber for joint control values
  ros::Subscriber m_radius_sub;
  ros::Subscriber m_amp_sub;
  ros::Subscriber m_omega_sub;
  ros::Subscriber m_theta_sub;
  ros::Subscriber m_lin_red_sub;
  std::vector<ros::Subscriber> m_pos_sub_vec;

  /// \brief keep track of controller update sim-time.
  private: double lastControllerUpdateTime;

  /// \brief Controller update mutex.
  private: std::mutex mutex;

  // Generic position command callback function (ROS topic)
  void positionCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint);

  // ROS joint controller parameter setter service
  private: ros::ServiceServer m_setPIDParameterService;
  
  float snake_module_length;
  
/* --------------------------- CPG --------------------------- */
  double derivate_phase(double phi_l, double phi, double phi_r); // temporal slope of the corresponding cpg node
  void update_phase(int idx, double elapsed_time); // runge-kutta approximation for the DE of the cpgs
  
  SecOrdState derivate_amplitude(const SecOrdState& r);
  SecOrdState derivate_bias(const SecOrdState& c);
  void update_cpg(double dt);
  void update_amplitude(double elapsed_time); // runge-kutta approximation for the DE of the cpgs
  void update_bias(double elapsed_time); // runge-kutta approximation for the DE of the cpgs
	
	float cpg_mu;     // convergence gain for phase
	float cpg_eps;    // tolerance for r and c (save update computation if value is close enough)
	float cpg_gain;   // convergence gain for r and c
	float cpg_Bias;   // desired bias
	float cpg_A;      // desired amplitude
	float cpg_omega;  // angular velocity
	float cpg_theta;  // desired shift between phases
	float snake_radius; // turning radius of the snake
	
	void compute_bias();
	
	std::vector<double> cpg_phase;  // the actual phase values of n cpg nodes, only touched by runge-kutta
	SecOrdState         cpg_a;
	SecOrdState         cpg_bias;
	
	float rk_h;
	float y, z;
};

} // namespace gazebo

#endif
