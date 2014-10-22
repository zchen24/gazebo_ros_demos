#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames_io.hpp>

// cisst
#include <cisstRobot.h>
#include <cisstVector.h>

std::ostream& operator << (std::ostream& os, const KDL::JntArray& J)
{
  for (unsigned int i = 0; i < J.rows(); i++) {
    os << J.data[i] << " ";
  }
  os << std::endl;
  return os;
}

namespace gazebo
{

class GravityCompensation : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    ROS_INFO("This is Model Plugin for GC testing");

    // ROS Nodehandle
    this->node = new ros::NodeHandle("~");
    this->pubJntStates = this->node->advertise<sensor_msgs::JointState>("/gravity/joint_states", 1000);
    this->sub = this->node->subscribe("/gravity/sub", 100, &GravityCompensation::callback, this);
    subJointDesired = node->subscribe("/gravity/joint_cmd", 100, &GravityCompensation::JointDesiredCallback, this);
    subPIDSwitch = node->subscribe("/gravity/pid_switch", 100, &GravityCompensation::PIDSwitchCallback, this);
    subCollect = node->subscribe("/gravity/collect", 100, &GravityCompensation::CollectCallback, this);

    // Store the pointer to the model
    this->model = _parent;//    RRBotKd

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GravityCompensation::OnUpdate, this, _1));

    // cisst robManipulator
    std::string robFileName = ros::package::getPath("rrbot_description");
    robFileName.append("/DH/rrbot.rob");
    robManipulator::Errno result;
    result = RRBotCisst.LoadRobot(robFileName);
    if (result == robManipulator::EFAILURE) {
        ROS_ERROR("cisst: failed to load manipulator config file: %s",
                  robFileName.c_str());
    } else {
        ROS_INFO("cisst: loaded rrbot manipulator");
    }        

    // Rotate the base
    vctMatrixRotation3<double> Rw0(  0.0,  0.0, -1.0,
                                     0.0,  1.0,  0.0,
                                     1.0,  0.0,  0.0 );
    vctFixedSizeVector<double,3> tw0(0.0);
    vctFrame4x4<double> Rtw0( Rw0, tw0 );
    RRBotCisst.Rtw0 = Rtw0;

    mNumJnts = RRBotCisst.links.size();
    q_des.SetSize(mNumJnts);
    q_des.SetAll(0.0);
    q_err_sum.SetSize(mNumJnts);
    q_err_sum.SetAll(0.0);

    // KDL from DH parameters
    // a, alpha, d, theta
    // m, com, RotationInertia(ixx, iyy, izz, ixy, ixz, iyz)
    KDL::RigidBodyInertia inert;

    inert = KDL::RigidBodyInertia(1.0, KDL::Vector(-0.45, 0, 0), KDL::RotationalInertia(1, 1, 1, 0, 0, 0));
    RRBotKdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.9, 0.0, 0.0, 0.0), inert));
    RRBotKdl.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.9, 0.0, 0.1, 0.0), inert));   

    // ---- PID -----
    pidEnabled = true;
    gcEnabled = true;
    collectEnabled = false;
  }

  // Called by the world update start event
public:
  void OnUpdate(const common::UpdateInfo & _info)
  {
    static int count = 0;
    count++;

    // read joint position
//    unsigned int mNumJnts = 2;
    vctDoubleVec q(mNumJnts, 0.0);
    vctDoubleVec qd(mNumJnts, 0.0);
    vctDoubleVec qdd(mNumJnts, 0.0);
    vctDoubleVec q_zero(mNumJnts, 0.0);
    vctDoubleVec taugc(mNumJnts, 0.0);
    vctDoubleVec taupid(mNumJnts, 0.0);
    vctDoubleVec tau(mNumJnts, 0.0);
    q[0] = this->model->GetJoint("joint1")->GetAngle(0).Radian();
    q[1] = this->model->GetJoint("joint2")->GetAngle(0).Radian();
    qd[0] = this->model->GetJoint("joint1")->GetVelocity(0);
    qd[1] = this->model->GetJoint("joint2")->GetVelocity(0);

    // compute FK
    vctFrm4x4 fkCisst;
    fkCisst = RRBotCisst.ForwardKinematics(q);
    taugc = RRBotCisst.InverseDynamics(q, q_zero, q_zero);

    KDL::ChainFkSolverPos_recursive fkSolver = KDL::ChainFkSolverPos_recursive(RRBotKdl);
    KDL::ChainIdSolver_RNE gcSolver = KDL::ChainIdSolver_RNE(RRBotKdl, KDL::Vector(-9.81, 0.0, 0.0));

    KDL::JntArray jnt_q(mNumJnts);
    KDL::JntArray jnt_qd(mNumJnts);
    KDL::JntArray jnt_qdd(mNumJnts);
    KDL::JntArray jnt_taugc(mNumJnts);
    KDL::Wrenches jnt_wrenches;
    for (unsigned int i = 0; i < mNumJnts; i++) {
      jnt_q(i) = q[i];
      jnt_qd(i) = 0.0;
      jnt_qdd(i) = 0.0;
      jnt_wrenches.push_back(KDL::Wrench());
    }
    KDL::Frame fkKDL;
    fkSolver.JntToCart(jnt_q, fkKDL);
    int ret;
    ret = gcSolver.CartToJnt(jnt_q, jnt_qd, jnt_qdd, jnt_wrenches,jnt_taugc);
    if (ret < 0) ROS_ERROR("KDL: inverse dynamics ERROR");

    // PID control
    double pgain[2] = {150.0, 150.0};
    double dgain[2] = {-15.0, -15.0};
    double igain[2] = {1.0, 1.0};

    if (pidEnabled)
    {
      double err_sum_limit = 10;

      for (unsigned i = 0; i < mNumJnts; i++) {
        taupid[i] = pgain[i] * (q_des[i] - q[i]) + dgain[i] * qd[i] + igain[i] * q_err_sum[i];
        q_err_sum[i] = q_err_sum[i] * 0.99 + (q_des[i] - q[i]);
        if (q_err_sum[i] > err_sum_limit) q_err_sum[i] = err_sum_limit;
        else if (q_err_sum[i] < -err_sum_limit) q_err_sum[i] = -err_sum_limit;
      }
      tau += taupid;
    }
    if (gcEnabled) tau += taugc;

    if (collectEnabled) {
        tau[0] = -0.6;
        tau[1] = -0.1;
    }

    this->model->GetJoint("joint1")->SetForce(0, tau[0]);
    this->model->GetJoint("joint2")->SetForce(0, tau[1]);

#if 1
    if (count%500 == 0) {
      vctQuatRot3 rotQuat(fkCisst.Rotation(), VCT_NORMALIZE);
      double x, y, z, w;
      fkKDL.M.GetQuaternion(x, y, z, w);


      ROS_INFO_STREAM(" " << std::endl
                      << "==================================" << std::endl
                      << "q = " << q << std::endl
                      << fkCisst.Translation() << std::endl
                      << "    " << rotQuat.X() << " " << rotQuat.Y() << " "
                      << rotQuat.Z() << " " << rotQuat.W() << std::endl
                      << "tau gc = " << taugc << std::endl
                      << "tau pid = " << taupid << std::endl
                      << "tau sum = " << tau << std::endl

                      << " ------- KDL ---------" << std::endl
                      << fkKDL.p << std::endl
                      << x << " " << y << " " << z << " " << w << std::endl
                      << "tau gc = " << jnt_taugc << std::endl;
                      );

//      std_msgs::Int16 msg;
//      msg.data = 5;
//      pub.publish(msg);
    }

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.position.clear(); msg.effort.clear();
    msg.position.push_back(q[0]); msg.position.push_back(q[1]);
    msg.effort.push_back(tau[0]); msg.effort.push_back(tau[1]);
    pubJntStates.publish(msg);
#endif

  }

  void callback(const std_msgs::Int16::ConstPtr &msg)
  {
    ROS_ERROR_STREAM("Message received data = " << msg->data);
  }

  void JointDesiredCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    if (msg->position.size() != q_des.size()) {
      ROS_ERROR("Desired joint position does NOT match robot joint size");
      return;
    }

    for (unsigned int i = 0; i < msg->position.size(); i++) {
      q_des[i] = msg->position.at(i);
    }

    ROS_INFO_STREAM("Jnt Cmd:" << q_des);
  }

  void PIDSwitchCallback(const std_msgs::BoolPtr &msg)
  {
    pidEnabled = msg->data;

    if (pidEnabled) ROS_INFO("PID ON");
    else ROS_INFO("PID OFF");
  }

  void CollectCallback(const std_msgs::Empty &msg)
  {
      collectEnabled = !collectEnabled;
  }

private:
  // Pointer to the model
  physics::ModelPtr model;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // Model info
  unsigned int mNumJnts;
  robManipulator RRBotCisst;
  KDL::Chain RRBotKdl;

  // ROS
  ros::NodeHandle* node;
  ros::Publisher pubJntStates;
  ros::Subscriber sub;
  ros::Subscriber subJointDesired;
  ros::Subscriber subPIDSwitch;
  ros::Subscriber subCollect;

  // Robot Control
  bool pidEnabled;      // pid enable switch
  bool gcEnabled;       // gc enable switch
  bool collectEnabled;  // collect enable switch
  vctDoubleVec q_des;   // desired joint position
  vctDoubleVec q_err_sum;  // sum of error
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GravityCompensation)
}
