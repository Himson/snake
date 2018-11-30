#include "wheel_less_snake_controller.h"

#include <boost/bind.hpp>
#include <ros/time.h>

namespace gazebo
{

SnakeCPGController::SnakeCPGController()
{
  m_nh = ros::NodeHandle();
}

SnakeCPGController::~SnakeCPGController()
{
  m_nh.shutdown();
}

void SnakeCPGController::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Store the pointer to the model
  m_model = parent;
  m_joint_controller = m_model->GetJointController();
  m_joint_map = m_joint_controller->GetJoints();
  
  ROS_INFO("sdf name %s, sdf description %s", sdf->GetName().c_str(), sdf->GetDescription().c_str());

  for (JointMap::iterator joint_iter = m_joint_map.begin(); joint_iter != m_joint_map.end(); ++joint_iter)
  {
    physics::JointPtr joint = joint_iter->second;

    sdf::ElementPtr sdf_ctrl_def;

    // check, if controller for current joint was specified in the SDF and return sdf element pointer to controller
    if (existsControllerSDF(sdf_ctrl_def, sdf, joint))
    {
      // get controller parameter from sdf file
      common::PID ctrl_pid = getControllerPID(sdf_ctrl_def);

      // create controller regarding the specified controller type
      createPositionController(joint, ctrl_pid);
      
      cpg_phase.push_back(0.0);
      m_joints.push_back(joint);
    }
  }

  // Controller time control.
  this->lastControllerUpdateTime = this->m_model->GetWorld()->GetSimTime().Double();

  // Listen to the update event. This event is broadcast every simulation iteration.
  m_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SnakeCPGController::OnUpdate, this, _1));

  m_setPIDParameterService = m_nh.advertiseService<
    SetPIDParameters::Request,
    SetPIDParameters::Response>(
      m_model->GetName() + "/set_pid_parameters",
      boost::bind(&SnakeCPGController::setPIDParametersCB, this, _1, _2)
  );
  
  // generate topic names using the model name as prefix
  std::string topic_name = m_model->GetName() + "/radius";
  replace(topic_name.begin(), topic_name.end(), ':', '_');
  m_radius_sub = m_nh.subscribe<std_msgs::Float64>(topic_name, 1, &SnakeCPGController::radiusCB, this);
  
  topic_name = m_model->GetName() + "/amplitude";
  replace(topic_name.begin(), topic_name.end(), ':', '_');
  m_amp_sub = m_nh.subscribe<std_msgs::Float64>(topic_name, 1, &SnakeCPGController::ampCB, this);
  
  topic_name = m_model->GetName() + "/vel";
  replace(topic_name.begin(), topic_name.end(), ':', '_');
  m_omega_sub = m_nh.subscribe<std_msgs::Float64>(topic_name, 1, &SnakeCPGController::omegaCB, this);
  
  topic_name = m_model->GetName() + "/phase_diff";
  replace(topic_name.begin(), topic_name.end(), ':', '_');
  m_theta_sub = m_nh.subscribe<std_msgs::Float64>(topic_name, 1, &SnakeCPGController::thetaCB, this);
  
  topic_name = m_model->GetName() + "/lin_red";
  replace(topic_name.begin(), topic_name.end(), ':', '_');
  m_lin_red_sub = m_nh.subscribe<std_msgs::Float64>(topic_name, 1, &SnakeCPGController::linRedCB, this);
  
  snake_module_length = 0.35;
  
  // initialize the cpg parameters
  cpg_mu = 5;
  cpg_gain = 2;
  cpg_eps = 0.005;
  cpg_theta = 1.5*pi/cpg_phase.size();
  cpg_omega = 0.0;
  cpg_bias.v1 = 0.0;
  cpg_bias.v2 = 0.0;
  cpg_a.v1 = 0.0;
  cpg_a.v2 = 0.0;
  cpg_Bias = 0.0;
  cpg_A = 0.0;
  
  rk_h = 0.05;
  z = 0.7;
  y = 1.0 - z;
}

// Called by the world update start event
void SnakeCPGController::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  double curTime = this->m_model->GetWorld()->GetSimTime().Double();
  double dt = curTime - this->lastControllerUpdateTime;

  if (dt >= 0.02) // update cpg states with 50 Hz
  {
    update_cpg(dt);
    
    this->lastControllerUpdateTime = curTime;
  }
}

void SnakeCPGController::update_cpg(double dt)
{
  update_amplitude(dt);
  update_bias(dt);
  
  /* for (int curIdx = 0; curIdx < m_joints.size(); curIdx++)
  {
    physics::JointPtr& joint = m_joints[curIdx];
    
    update_phase(curIdx, dt);
    
    double P = z * (double)(curIdx+1)/m_joints.size() + y;
    
    double angle_rad = cpg_bias.v1 + P * cpg_a.v1 * sin(cpg_phase[curIdx]);

    m_joint_controller->SetPositionTarget(joint->GetScopedName(), angle_rad);
  } */
  
  // ----------  Head Compensation algo  -----------
  
  update_phase(0, dt);	// update pattern node of head joint
  
  double theta = 0;	// theta is the angle between the head module and the current module
  double alpha = theta;	// alpha is the sum over all theta. It is related to the heading direction.

  for (int curIdx = 0; curIdx < m_joints.size(); curIdx++)
  {
    physics::JointPtr& joint = m_joints[curIdx];
    
    update_phase(curIdx, dt);
    
    double P = z * (double)(curIdx)/m_joints.size() + y;
    
    double phi = cpg_bias.v1 + P * cpg_a.v1 * cos(cpg_phase[curIdx]);

    m_joint_controller->SetPositionTarget(joint->GetScopedName(), phi);
    ROS_INFO("Joint %d position: %f rad phase: %f amp: %f", curIdx+1, phi, cpg_phase[curIdx], cpg_A);
    
    theta += joint->GetAngle(0).Radian();
    alpha += theta;
  }
  
  alpha /= m_joints.size();

}

void SnakeCPGController::update_phase(int idx, double elapsed_time)
{
  if(idx < 0 || idx >= cpg_phase.size())
    return;
   
  double phi_l,phi,phi_r; 
  if(idx == 0)
  {
    phi   = cpg_phase[idx];
    phi_l = phi + cpg_theta;
    phi_r = cpg_phase[idx+1];
  }
  else if(idx == cpg_phase.size()-1)
  {
    phi   = cpg_phase[idx];
    phi_l = cpg_phase[idx-1];
    phi_r = phi - cpg_theta;
  }
  else
  {
    phi   = cpg_phase[idx];
    phi_l = cpg_phase[idx-1];
    phi_r = cpg_phase[idx+1];
  }

  double time = 0.0;
  
  while(time < elapsed_time)
  {
    double k1 = derivate_phase(phi_l, phi              , phi_r);
    double k2 = derivate_phase(phi_l, phi + 0.5*rk_h*k1, phi_r);
    double k3 = derivate_phase(phi_l, phi + 0.5*rk_h*k2, phi_r);
    double k4 = derivate_phase(phi_l, phi +     rk_h*k3, phi_r);

    phi = phi + rk_h/6 * (k1 + 2*k2 + 2*k3 + k4);
    
    time += rk_h;
  }
  
  cpg_phase[idx] = phi;
}

void SnakeCPGController::update_amplitude(double elapsed_time)
{
  if(std::abs(cpg_a.v1 - cpg_A) < cpg_eps)
    return;
   
  double time = 0.0;
  SecOrdState tmp;
  while(time < elapsed_time)
  {
    SecOrdState k1 = derivate_amplitude(cpg_a);
    tmp.v1 = cpg_a.v1 + 0.5*rk_h*k1.v1; tmp.v2 = cpg_a.v2 + 0.5*rk_h*k1.v2;
    SecOrdState k2 = derivate_amplitude(tmp);
    tmp.v1 = cpg_a.v1 + 0.5*rk_h*k2.v1; tmp.v2 = cpg_a.v2 + 0.5*rk_h*k2.v2;
    SecOrdState k3 = derivate_amplitude(tmp);
    tmp.v1 = cpg_a.v1 + rk_h*k3.v1; tmp.v2 = cpg_a.v2 + rk_h*k3.v2;
    SecOrdState k4 = derivate_amplitude(tmp);

    cpg_a.v1 = cpg_a.v1 + rk_h/6 * (k1.v1 + 2*k2.v1 + 2*k3.v1 + k4.v1);
    cpg_a.v2 = cpg_a.v2 + rk_h/6 * (k1.v2 + 2*k2.v2 + 2*k3.v2 + k4.v2);
    
    time += rk_h;
  }
}

void SnakeCPGController::update_bias(double elapsed_time)
{
  if(std::abs(cpg_bias.v1 - cpg_Bias) < cpg_eps)
      return;
     
  double time = 0.0;
  SecOrdState tmp;
  while(time < elapsed_time)
  {
    SecOrdState k1 = derivate_bias(cpg_bias);
    tmp.v1 = cpg_bias.v1 + 0.5*rk_h*k1.v1; tmp.v2 = cpg_bias.v2 + 0.5*rk_h*k1.v2;
    SecOrdState k2 = derivate_bias(tmp);
    tmp.v1 = cpg_bias.v1 + 0.5*rk_h*k2.v1; tmp.v2 = cpg_bias.v2 + 0.5*rk_h*k2.v2;
    SecOrdState k3 = derivate_bias(tmp);
    tmp.v1 = cpg_bias.v1 + rk_h*k3.v1; tmp.v2 = cpg_bias.v2 + rk_h*k3.v2;
    SecOrdState k4 = derivate_bias(tmp);

    cpg_bias.v1 = cpg_bias.v1 + rk_h/6 * (k1.v1 + 2*k2.v1 + 2*k3.v1 + k4.v1);
    cpg_bias.v2 = cpg_bias.v2 + rk_h/6 * (k1.v2 + 2*k2.v2 + 2*k3.v2 + k4.v2);
    
    time += rk_h;
  }
}

double SnakeCPGController::derivate_phase(double phi_l, double phi, double phi_r)
{ // hand-crafted differential equation for the phases. It minimzes the error between theta and theta_desired
  return cpg_omega + cpg_mu * (phi_l - 2*phi + phi_r);
}

SecOrdState SnakeCPGController::derivate_amplitude(const SecOrdState& a)
{
  SecOrdState da;
  da.v1 = a.v2;
  da.v2 = cpg_gain * (cpg_gain/4.0 * (cpg_A - a.v1) - a.v2);
  return da;
}
SecOrdState SnakeCPGController::derivate_bias(const SecOrdState& bias)
{
  SecOrdState dc;
  dc.v1 = bias.v2;
  dc.v2 = cpg_gain * (cpg_gain/4.0 * (cpg_Bias - bias.v1) - bias.v2);
  return dc;
}

void SnakeCPGController::compute_bias()
{
  int K = cpg_phase.size(); // number of modules

  float theta[K+1];
  theta[0] = 0.0;
  float head_dir = 0.0; 
  for(int i = 1; i < K+1; i++)
  {
  	theta[i] = theta[i-1] + cpg_A * cos(cpg_theta * i);
  	head_dir += theta[i];
  }
  
  head_dir /= K+1;
  
  float l = 0;
  for(int i = 0; i < K+1; i++)
  {
  	l += cos(theta[i] - head_dir);
  }
  l *= snake_module_length;
  
  if(abs(snake_radius) <= 0.5)
    cpg_Bias = 0;
  else
    cpg_Bias = l/(snake_radius * (K-1));
  
  ROS_DEBUG("Computed new bias: amp=%f | theta=%f | radius=%f | arcLength=%f | bias=%f", cpg_A, cpg_theta, snake_radius, l, cpg_Bias);
}

///////////////////////////////////////// SDF parser functions ////////////////////////////////////////////

bool SnakeCPGController::existsControllerSDF(sdf::ElementPtr &sdf_ctrl_def, const sdf::ElementPtr &sdf,
                                               const physics::JointPtr &joint)
{
  sdf::ElementPtr sdf_ctrl = sdf->GetElement("controller");
  while (sdf_ctrl != NULL)
  {
    sdf::ParamPtr joint_name_attr = sdf_ctrl->GetAttribute("joint_name");
    if (joint_name_attr != NULL)
    {
      std::string joint_name = joint_name_attr->GetAsString();
      if (joint_name == joint->GetName())
      {
        ROS_INFO("Found controller for joint %s", joint_name.c_str());
        sdf_ctrl_def = sdf_ctrl;
        return true;
      }
      else
      {
        // find next controller
        sdf_ctrl = sdf_ctrl->GetNextElement("controller");
      }
    }
    else
    {
      ROS_WARN("Attribute 'joint_name' is not available for current controller.");
      return false;
    }
  }

  ROS_WARN("No controller for joint %s found", joint->GetName().c_str());
  return false;
}

common::PID SnakeCPGController::getControllerPID(const sdf::ElementPtr &sdf_ctrl_def)
{
  common::PID pid_param;

  if (sdf_ctrl_def != NULL)
  {
    sdf::ElementPtr elem_pid = sdf_ctrl_def->GetElement("pid");
    if (elem_pid != NULL)
    {
#if SDF_MAJOR_VERSION > 3
      ignition::math::Vector3d pid_values = elem_pid->Get<ignition::math::Vector3d>();
      ROS_INFO("Controller PID values p=%f, i=%f, d=%f", pid_values.X(), pid_values.Y(), pid_values.Z());
      pid_param = common::PID(pid_values.X(), pid_values.Y(), pid_values.Z());
#else
      sdf::Vector3 pid_values = elem_pid->Get<sdf::Vector3>();
      ROS_INFO("Controller PID values p=%f, i=%f, d=%f", pid_values.x, pid_values.y, pid_values.z);
      pid_param = common::PID(pid_values.x, pid_values.y, pid_values.z);
#endif
      return pid_param;
    }
  }

  ROS_WARN("Could not find controller PID parameter in SDF file: Using default values.");
  pid_param = common::PID(1.0, 0.1, 0.01);
  return pid_param;
}

//////////////////////////////////////// Controller construction //////////////////////////////////////////

void SnakeCPGController::createPositionController(const physics::JointPtr &joint, const common::PID &pid_param)
{
  // generate joint topic name using the model name as prefix
  std::string topic_name = m_model->GetName() + "/" + joint->GetName() + "/cmd_pos";
  replace(topic_name.begin(), topic_name.end(), ':', '_');

  // Add ROS topic for position control
  m_pos_sub_vec.push_back(m_nh.subscribe<std_msgs::Float64>(topic_name, 1,
                                                            boost::bind(&SnakeCPGController::positionCB, this, _1, joint)));
  
  // Create PID parameter for position controller
  m_joint_controller->SetPositionPID(joint->GetScopedName(), pid_param);

  // Initialize controller with zero position
  m_joint_controller->SetPositionTarget(joint->GetScopedName(), 0.0);

  ROS_INFO("Added new position controller for joint %s", joint->GetName().c_str());
}

//////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////

void SnakeCPGController::positionCB(const std_msgs::Float64::ConstPtr &msg, const physics::JointPtr &joint)
{
  ROS_DEBUG("positionCB called! Joint name = %s, joint pos = %f", joint->GetName().c_str(), msg->data);

  double angle_rad(msg->data);
  //m_joint_controller->SetPositionTarget(joint->GetScopedName(), angle_rad);
}

void SnakeCPGController::radiusCB(const std_msgs::Float64::ConstPtr &msg)
{
  double m(msg->data);
  ROS_DEBUG("radiusCB called! Data: %f", m);
  snake_radius = m;
  
  compute_bias();
}
void SnakeCPGController::ampCB(const std_msgs::Float64::ConstPtr &msg)
{
  double m(msg->data);
  ROS_DEBUG("ampCB called! Data: %f", m);
  cpg_A = m;

  compute_bias();
}
void SnakeCPGController::omegaCB(const std_msgs::Float64::ConstPtr &msg)
{
  double m(msg->data);
  ROS_DEBUG("omegaCB called! Data: %f", m);
  cpg_omega = m;
}
void SnakeCPGController::thetaCB(const std_msgs::Float64::ConstPtr &msg)
{
  double m(msg->data);
  ROS_DEBUG("thetaCB called! Data: %f", m);
  //cpg_theta = m*pi/cpg_phase.size();
  cpg_theta = m;

  compute_bias();
}
void SnakeCPGController::linRedCB(const std_msgs::Float64::ConstPtr &msg)
{
  double m(msg->data);
  ROS_DEBUG("linRedCB called! Data: %f", m);
  
  if(m >= 0.0){
  	z = m;
  	y = 1.0 - z;
  }
}

bool SnakeCPGController::setPIDParametersCB(SetPIDParameters::Request &req,
                                              SetPIDParameters::Response &res)
{
  std::string joint_name = req.joint;
  double kp = req.kp,
         ki = req.ki,
         kd = req.kd;
  ROS_DEBUG("setPIDParametersCB called! Joint name = %s, kp = %f, ki = %f, kd = %f", joint_name.c_str(), kp, ki, kd);

  JointMap::iterator it = m_joint_map.find(joint_name);
  if (it == m_joint_map.end())
  {
    res.success = false;
    return false;
  }
  else
  {
    const std::string joint_id = it->second->GetScopedName();
    m_joint_controller->SetPositionPID(joint_id, common::PID(kp, ki, kd));
    res.success = true;
    return true;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SnakeCPGController)

} // namespace gazebo
