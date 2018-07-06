/**
 * @file WalkingModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>
#include <memory>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>
#include "iDynTree/yarp/YARPConfigurationsLoader.h"

#include "WalkingModule.hpp"
#include "Utils.hpp"

void WalkingModule::propagateTime()
{
    // propagate time
    m_time += m_dT;
}

bool WalkingModule::propagateReferenceSignals()
{
    // check if vector is not initialized
    if(m_leftTrajectory.empty()
       || m_rightTrajectory.empty()
       || m_leftInContact.empty()
       || m_rightInContact.empty()
       || m_DCMPositionDesired.empty()
       || m_DCMVelocityDesired.empty()
       || m_comHeightTrajectory.empty())
    {
        yError() << "[propagateReferenceSignals] Cannot propagate empty reference signals.";
        return false;
    }

    m_rightTrajectory.pop_front();
    m_rightTrajectory.push_back(m_rightTrajectory.back());

    m_leftTrajectory.pop_front();
    m_leftTrajectory.push_back(m_leftTrajectory.back());

    m_rightTwistTrajectory.pop_front();
    m_rightTwistTrajectory.push_back(m_rightTwistTrajectory.back());

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if(!m_mergePoints.empty())
    {
        for(auto& mergePoint : m_mergePoints)
            mergePoint--;

        if(m_mergePoints[0] == 0)
            m_mergePoints.pop_front();
    }
    return true;
}

double WalkingModule::getPeriod()
{
    //  period of the module (seconds)
    return m_dT;
}

bool WalkingModule::setControlledJoints(const yarp::os::Searchable& rf)
{
    // get joints list from resource finder
    yarp::os::Value *axesListYarp;
    if(!rf.check("joints_list", axesListYarp))
    {
        yError() << "[setControlledJoints] Unable to find joints_list into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[setControlledJoints] Unable to convert yarp list into a vector of strings.";
        return false;
    }
    return true;
}

bool WalkingModule::setRobotModel(const yarp::os::Searchable& rf)
{
    if(m_axesList.empty())
    {
        yError() << "[setRobotModel] The list containing the controlled joints is empty. "
                 <<  "Please call setControlledJoints()";
        return false;
    }

    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model",yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    yInfo() << "The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_axesList))
    {
        yError() << "[setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    
    return true;
}

bool WalkingModule::configureRobot(const yarp::os::Searchable& rf)
{
    // robot name: used to connect to the robot
    m_robot = rf.check("robot", yarp::os::Value("icubSim")).asString();

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value *iCubPartsYarp;
    if(!rf.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[configureRobot] Unable to find remote_control_boards into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    m_remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = m_remoteControlBoards.addList();
    for(auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + m_robot + "/" + iCubPart);

    options.put("remoteControlBoards", m_remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + getName() + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = m_axesList.size();

    // open the device
    if(!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    // obtain the interfaces
    if(!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[configureRobot] Cannot obtain IEncoders interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionControl interface";
        return false;
    }

    if(!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[configureRobot] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if(!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_limitsInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_loader.isValid())
    {
        yError() << "[configureRobot] The iDynTree model loader is not valid. "
                 << "Have you already called 'setRobotModel()' method.";
        return false;
    }

    // resize the buffers
    m_positionFeedbackInDegrees.resize(m_actuatedDOFs, 0.0);
    m_velocityFeedbackInDegrees.resize(m_actuatedDOFs, 0.0);

    m_positionFeedbackInRadians.resize(m_actuatedDOFs);
    m_velocityFeedbackInRadians.resize(m_actuatedDOFs);
    m_qDesired.resize(m_actuatedDOFs);
    m_dqDesired_osqp.resize(m_actuatedDOFs);
    m_dqDesired_qpOASES.resize(m_actuatedDOFs);
    m_toDegBuffer.resize(m_actuatedDOFs);
    m_minJointsLimit.resize(m_actuatedDOFs);
    m_maxJointsLimit.resize(m_actuatedDOFs);

    // m_positionFeedbackInDegreesFiltered.resize(m_actuatedDOFs);
    // m_positionFeedbackInDegreesFiltered.zero();

    m_velocityFeedbackInDegreesFiltered.resize(m_actuatedDOFs);
    m_velocityFeedbackInDegreesFiltered.zero();

    // check if the robot is alive
    bool okPosition = false;
    bool okVelocity = false;
    for (int i=0; i < 10 && !okPosition && !okVelocity; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());
        okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackInDegrees.data());

        if(!okPosition || !okVelocity)
            yarp::os::Time::delay(0.1);
    }
    if(!okPosition)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    if(!okVelocity)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    // set the inertial to world rotation
    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();

    m_useVelocityFilter = rf.check("use_joint_velocity_filter", yarp::os::Value("False")).asBool();
    if(m_useVelocityFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getDoubleFromSearchable(rf, "joint_velocity_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        // set filters
        // m_positionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(10, m_dT);
        m_velocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, m_dT);

        // m_positionFilter->init(m_positionFeedbackInDegrees);
        m_velocityFilter->init(m_velocityFeedbackInDegrees);
    }

    m_useWrenchFilter = rf.check("use_wrench_filter", yarp::os::Value("False")).asBool();
    if(m_useWrenchFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getDoubleFromSearchable(rf, "wrench_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        m_leftWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, m_dT);
        m_rightWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, m_dT);
    }

    // get the limits
    double max, min;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        if(!m_limitsInterface->getVelLimits(i, &min, &max))
        {
            yError() << "[configure] Unable get joints velocity limits.";
            return false;
        }

        m_minJointsLimit(i) = -iDynTree::deg2rad(max);
        m_maxJointsLimit(i) = iDynTree::deg2rad(max);
    }
    return true;
}

bool WalkingModule::configureForceTorqueSensors(const yarp::os::Searchable& config)
{
    std::string portInput, portOutput;

    // check if the config file is empty
    if(config.isNull())
    {
        yError() << "[configureForceTorqueSensors] Empty configuration for the force torque sensors.";
        return false;
    }

    // open and connect left foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchInputPort_name", portInput))
    {
        yError() << "[configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_leftWrenchPort.open("/" + getName() + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + getName() + portInput))
    {
        yError() << "Unable to connect to port " << "/" + getName() + portInput;
        return false;
    }

    // open and connect right foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchInputPort_name", portInput))
    {
        yError() << "[configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_rightWrenchPort.open("/" + getName() + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + getName() + portInput))
    {
        yError() << "Unable to connect to port " << "/" + getName() + portInput;
        return false;
    }

    return true;
}

bool WalkingModule::configureIMU(const yarp::os::Searchable& config)
{
    // check IMU configurations
    m_useHeadIMU = config.check("useHeadIMU", yarp::os::Value(false)).asBool();
    m_useFeetIMU = config.check("useFeetIMU", yarp::os::Value(false)).asBool();
    m_useIMUDS = config.check("useIMUDS", yarp::os::Value(false)).asBool();
    m_ignoreYaw = config.check("ignoreYaw", yarp::os::Value(false)).asBool();
    
    if(m_useIMUDS)
      yInfo() << "Using IMU only in double support.";
    
    if(!m_useFeetIMU && !m_useHeadIMU)
    {
      yInfo() << "All IMUs set to false, please check your options!";
      return false;
    }
    
    if(m_useHeadIMU)
    {
      m_HeadIMUPort.open("/" + getName() + "/inertial:i");
      std::string headPortName = config.check("IMUHeadPort", yarp::os::Value("/inertial")).asString();
      if(!yarp::os::Network::connect("/" + m_robot + headPortName, "/" + getName() + "/inertial:i"))
      {
        yError() << "Unable to connect to port " << "/" + m_robot + headPortName;
        return false;
      }
      
      m_imuHeadFrame = config.check("IMUHead", yarp::os::Value("imu_frame")).asString();
      
      yInfo() << "Using head IMU.";
    }
    
    if(m_useFeetIMU)
    {
      std::string RPortName = config.check("IMURightFootPort", yarp::os::Value("/right_foot/imu/measures:o")).asString();
      std::string LPortName = config.check("IMULeftFootPort", yarp::os::Value("/left_foot/imu/measures:o")).asString();
      if(m_robot.compare("icubSim") == 0)
      {
        m_RFootIMUPortSim.open("/" + getName() + "/right_foot/measures:i");
        m_LFootIMUPortSim.open("/" + getName() + "/left_foot/measures:i");
      }
      else 
      {
        m_RFootIMUPort.open("/" + getName() + "/right_foot/measures:i");
        m_LFootIMUPort.open("/" + getName() + "/left_foot/measures:i");
      }
      if(!yarp::os::Network::connect("/" + m_robot + RPortName, "/" + getName() + "/right_foot/measures:i"))
      {
        yError() << "Unable to connect to port " << "/" + m_robot + RPortName;
        return false;
      }
      if(!yarp::os::Network::connect("/" + m_robot + LPortName, "/" + getName() + "/left_foot/measures:i"))
      {
        yError() << "Unable to connect to port " << "/" + m_robot + LPortName;
        return false;
      }
      
      m_imuRFootFrame = config.check("IMURightFoot", yarp::os::Value("r_foot_ft_sensor")).asString();
      m_imuLFootFrame = config.check("IMULeftFoot", yarp::os::Value("l_foot_ft_sensor")).asString();
      
      if(m_robot.compare("icubSim") != 0)
      {
        if(!iDynTree::parseRotationMatrix(config, "IMUToFT", m_IMUToFT)){
          m_IMUToFT = iDynTree::Rotation::Identity();
          yInfo() << "Using the identity as rotation between IMU and FT.";
        }
      } 
      else
        m_IMUToFT = iDynTree::Rotation::Identity();
      
      yInfo() << "Using feet IMU.";
    }
    
    if(m_useFeetIMU || m_useHeadIMU)
    {
      m_IMUThresholdPitch = iDynTree::deg2rad(config.check("tiltThresholdPitch", yarp::os::Value(1)).asDouble());
      m_IMUThresholdRoll = iDynTree::deg2rad(config.check("tiltThresholdRoll", yarp::os::Value(1)).asDouble());
      m_IMUPlaneThreshold = iDynTree::deg2rad(config.check("planeThreshold", yarp::os::Value(1)).asDouble());
      m_IMUWorldThresholdPitch = iDynTree::deg2rad(config.check("worldThresholdPitch", yarp::os::Value(1)).asDouble());
      m_IMUWorldThresholdRoll = iDynTree::deg2rad(config.check("worldThresholdRoll", yarp::os::Value(1)).asDouble());
      m_IMUSmoothingTime = config.check("smoothingTime", yarp::os::Value(0.5)).asDouble();
      m_useIMUFiltering = config.check("useIMUFiltering", yarp::os::Value(true)).asBool();
      m_updateOnceDS = config.check("updateOnceDS", yarp::os::Value(true)).asBool();
      m_useBiasMethod = config.check("useBiasMethod", yarp::os::Value(false)).asBool();
      
      if(!m_useIMUDS && m_updateOnceDS)
      {
        yInfo() << "[WARNING] Impossible to use update once if not using IMU only in DS. Deactivating.";
        m_updateOnceDS = false;
      }
      
      if(m_useIMUFiltering)
      {
        m_IMUFilterFreq = config.check("filterFreq", yarp::os::Value(100)).asDouble();
        if(m_useHeadIMU)
          m_HeadIMUFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(m_IMUFilterFreq, m_dT);
        if(m_useFeetIMU)
        {
          m_RFootIMUFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(m_IMUFilterFreq, m_dT);
          m_LFootIMUFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(m_IMUFilterFreq, m_dT);
        }
      }
      
      m_useFTDetection = config.check("useFTDetection", yarp::os::Value(false)).asBool();
      if(m_useFTDetection)
      {
        yInfo() << "Using FT fot contact detection.";
        m_FTThreshold = config.check("FTThreshold", yarp::os::Value(80)).asDouble();
        m_forcesThreshold = config.check("FTForcesThreshold", yarp::os::Value(10)).asDouble();
        m_velThreshold = config.check("velocityThreshold", yarp::os::Value(0.001)).asDouble();
        m_useSkin = config.check("useSkinDetection", yarp::os::Value(false)).asBool();
      }
      
      // Initialize data
      m_rotRFTToSole = iDynTree::Rotation::Identity();
      m_rotLFTToSole = iDynTree::Rotation::Identity();
      m_rotRFootIMU = iDynTree::Rotation::Identity();
      m_rotLFootIMU = iDynTree::Rotation::Identity();
      m_IMUToFT = iDynTree::Rotation::Identity();
      m_rotLEarthToWorld = iDynTree::Rotation::Identity(); // orientation between the inertial of the earth according to the IMU and the world frame
      m_rotREarthToWorld = iDynTree::Rotation::Identity();
      m_HeadIMUData.resize(3);
      m_HeadIMUData.zero();
      m_HeadIMUDataFilt.resize(3);
      m_HeadIMUDataFilt.zero();
      m_RFootIMUData.resize(3);
      m_LFootIMUData.resize(3);
      m_RFootIMUData.zero();
      m_LFootIMUData.zero();
      m_RFootIMUDataFilt.resize(3);
      m_LFootIMUDataFilt.resize(3);
      m_RFootIMUDataFilt.zero();
      m_LFootIMUDataFilt.zero();
//       if(m_useFTDetection || m_useSkin)
//       {
//         m_walkingStatus = WalkingStatus::Unknown;
//         m_prevWalkingStatus = m_walkingStatus;
//       }
//       else
//       {
//         m_walkingStatus = WalkingStatus::DS;
//         m_prevWalkingStatus = WalkingStatus::LSS;
//       }
      m_walkingStatus = WalkingStatus::DSStable;
      m_prevWalkingStatus = WalkingStatus::LSS;
      m_planeKx = 0;
      m_planeKy = 0;
      m_desiredZMPX = 0;
      m_desiredZMPY = 0;
      m_ortChanged = false;
      m_DSSwitchedOut = false;
      m_inertial_R_worldFrame_new = m_inertial_R_worldFrame;
      m_inertial_R_worldFrame_vec.resize(floor(m_IMUSmoothingTime/m_dT));
      for(int i = 0; i < m_inertial_R_worldFrame_vec.size(); i++)
        m_inertial_R_worldFrame_vec[i] = m_inertial_R_worldFrame;
      m_ortChangeIndex = floor(m_IMUSmoothingTime/m_dT);
      m_nominalWorldPitch = 0;
      m_nominalWorldRoll = 0;
    }
    
    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder& rf)
{
    // module name (used as prefix for opened ports)
    std::string string;
    if(!YarpHelper::getStringFromSearchable(rf, "name", string))
    {
        yError() << "[configure] Unable to get the string from searchable.";
        return false;
    }
    setName(string.c_str());

    m_useMPC = rf.check("use_mpc", yarp::os::Value(false)).asBool();
    m_useQPIK = rf.check("use_QP-IK", yarp::os::Value(false)).asBool();
    m_useOSQP = rf.check("use_osqp", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();

    if(!setControlledJoints(rf))
    {
        yError() << "[configure] Unable to set the controlled joints.";
        return false;
    }

    if(!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    if(!configureRobot(rf))
    {
        yError() << "[configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    yarp::os::Bottle& forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    if(!configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "Could not open" << rpcPortName.c_str() << "RPC port.";
        return false;
    }

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryGenerator>();
    yarp::os::Bottle& trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER");
    trajectoryPlannerOptions.append(generalOptions);
    if(!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }
    
    yarp::os::Bottle& IMUOptions = rf.findGroup("IMU");
    if(IMUOptions.size() > 0)
    {
      if(!configureIMU(IMUOptions))
      {
        yError() << "[configure] Unable to configure the IMU.";
        return false;
      }
    } else
    {
      m_useFeetIMU = false;
      m_useHeadIMU = false;
      m_useIMUDS = false;
      m_useIMUFiltering = false;
    }

    if(m_useMPC)
    {
        // initialize the MPC controller
        m_walkingController = std::make_unique<WalkingController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_MPC_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingController->initialize(dcmControllerOptions))
        {
            yError() << "[configure] Unable to initialize the controller.";
            return false;
        }
    }
    else
    {
        // initialize the MPC controller
        m_walkingDCMReactiveController = std::make_unique<WalkingDCMReactiveController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_REACTIVE_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingDCMReactiveController->initialize(dcmControllerOptions))
        {
            yError() << "[configure] Unable to initialize the controller.";
            return false;
        }
    }

    // initialize the ZMP controller
    m_walkingZMPController = std::make_unique<WalkingZMPController>();
    yarp::os::Bottle& zmpControllerOptions = rf.findGroup("ZMP_CONTROLLER");
    zmpControllerOptions.append(generalOptions);
    if(!m_walkingZMPController->initialize(zmpControllerOptions))
    {
        yError() << "[configure] Unable to initialize the ZMP controller.";
        return false;
    }

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WalkingIK>();
    yarp::os::Bottle& inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if(!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(), m_axesList))
    {
        yError() << "[configure] Failed to configure the ik solver";
        return false;
    }

    if(m_useQPIK)
    {
        yarp::os::Bottle& inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");

        m_QPIKSolver_osqp = std::make_unique<WalkingQPIK_osqp>();
        if(!m_QPIKSolver_osqp->initialize(inverseKinematicsQPSolverOptions,
                                          m_actuatedDOFs,
                                          m_minJointsLimit, m_maxJointsLimit))
        {
            yError() << "[configure] Failed to configure the QP-IK solver (osqp)";
            return false;
        }

        m_QPIKSolver_qpOASES = std::make_unique<WalkingQPIK_qpOASES>();
        if(!m_QPIKSolver_qpOASES->initialize(inverseKinematicsQPSolverOptions,
                                             m_actuatedDOFs,
                                             m_minJointsLimit, m_maxJointsLimit))
        {
            yError() << "[configure] Failed to configure the QP-IK solver (qpOASES)";
            return false;
        }
    }

    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle& forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if(!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[configure] Failed to configure the fk solver";
        return false;
    }

    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<StableDCMModel>();
    if(!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[configure] Failed to configure the lipm.";
        return false;
    }

    // set PIDs gains
    m_PIDHandler = std::make_unique<WalkingPIDHandler>();
    yarp::os::Bottle& pidOptions = rf.findGroup("PID");
    if (!m_PIDHandler->initialize(pidOptions, m_robotDevice, m_remoteControlBoards))
    {
        yError() << "[configure] Failed to configure the PIDs.";
        return false;
    }

    // initialize the logger
    if(m_dumpData)
    {
        m_walkingLogger = std::make_unique<WalkingLogger>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, getName()))
        {
            yError() << "[configure] Unable to configure the logger.";
            return false;
        }
    }

    // time profiler
    m_profiler = std::make_unique<TimeProfiler>();
    m_profiler->setPeriod(round(0.1 / m_dT));
    if(m_useMPC)
        m_profiler->addTimer("MPC");

    m_profiler->addTimer("IK");
    m_profiler->addTimer("Total");

    // initialize some variables
    m_firstStep = false;
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    yInfo() << "[configure] Ready to play!";

    return true;
}

bool WalkingModule::close()
{
    // set position control when the module is closed
    if(!switchToControlMode(VOCAB_CM_POSITION))
    {
        yError() << "[close] Unable to switch in position control.";
        return false;
    }

    if(m_dumpData)
        m_walkingLogger->quit();

    // restore PID
    m_PIDHandler->restorePIDs();

    // close the driver
    if(!m_robotDevice.close())
        yError() << "[close] Unable to close the device.";

    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_walkingController.reset(nullptr);
    m_walkingZMPController.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_QPIKSolver_osqp.reset(nullptr);
    m_QPIKSolver_qpOASES.reset(nullptr);
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);
    m_PIDHandler.reset(nullptr);
    m_leftWrenchFilter.reset(nullptr);
    m_rightWrenchFilter.reset(nullptr);
    m_positionFilter.reset(nullptr);
    m_velocityFilter.reset(nullptr);
    
    if(m_useIMUFiltering)
    {
      if(m_useHeadIMU)
        m_HeadIMUFilter.reset(nullptr);
      
      if(m_useFeetIMU)
      {
        m_RFootIMUFilter.reset(nullptr);
        m_LFootIMUFilter.reset(nullptr);
      }
    }

    // close the ports
    m_rpcPort.close();
    m_rightWrenchPort.close();
    m_leftWrenchPort.close();
    
    if(m_useHeadIMU)
      m_HeadIMUPort.close();
    
    if(m_useFeetIMU)
    {
      if(m_robot.compare("icubSim") == 0)
      {
        m_RFootIMUPortSim.close();
        m_LFootIMUPortSim.close();
      }
      else
      {
        m_RFootIMUPort.close();
        m_LFootIMUPort.close();
      }
    }

    return true;
}

bool WalkingModule::solveQPIK(auto& solver, const iDynTree::Position& desiredCoMPosition,
                              const iDynTree::Vector3& desiredCoMVelocity,
                              const iDynTree::Position& actualCoMPosition,
                              const iDynTree::Rotation& desiredNeckOrientation,
                              iDynTree::VectorDynSize &output)
{
    if(!solver->setRobotState(m_positionFeedbackInRadians,
                              m_FKSolver->getLeftFootToWorldTransform(),
                              m_FKSolver->getRightFootToWorldTransform(),
                              m_FKSolver->getNeckOrientation(),
                              actualCoMPosition))
    {
        yError() << "[solveQPIK] Unable to update the QP-IK solver";
        return false;
    }

    solver->setDesiredNeckOrientation(desiredNeckOrientation.inverse());

    solver->setDesiredFeetTransformation(m_leftTrajectory.front(),
                                         m_rightTrajectory.front());

    solver->setDesiredFeetTwist(m_leftTwistTrajectory.front(),
                                m_rightTwistTrajectory.front());

    solver->setDesiredCoMVelocity(desiredCoMVelocity);

    solver->setDesiredCoMPosition(desiredCoMPosition);

    // set jacobians
    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_actuatedDOFs + 6);
    comJacobian.resize(3, m_actuatedDOFs + 6);

    m_FKSolver->getLeftFootJacobian(jacobian);
    solver->setLeftFootJacobian(jacobian);

    m_FKSolver->getRightFootJacobian(jacobian);
    solver->setRightFootJacobian(jacobian);

    m_FKSolver->getNeckJacobian(jacobian);
    solver->setNeckJacobian(jacobian);

    m_FKSolver->getCoMJacobian(comJacobian);
    solver->setCoMJacobian(comJacobian);

    if(!solver->solve())
    {
        yError() << "[solveQPIK] Unable to solve the QP-IK problem.";
        return false;
    }

    if(!solver->getSolution(output))
    {
        yError() << "[solveQPIK] Unable to get the QP-IK problem solution.";
        return false;
    }

    return true;
}

bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState == WalkingFSM::Walking
       || m_robotState == WalkingFSM::Stance
       || m_robotState == WalkingFSM::OnTheFly)
    {
      
        iDynTree::Vector2 measuredDCM, measuredZMP;
        iDynTree::Position measuredCoM;
        iDynTree::Vector3 measuredCoMVelocity;

        bool resetTrajectory = false;

        m_profiler->setInitTime("Total");

        // if a new trajectory is required check if its the time to evaluate the new trajectory or
        // the time to attach new one
        if(m_newTrajectoryRequired)
        {
            // when we are near to the merge point the new trajectory is evaluated
            if(m_newTrajectoryMergeCounter == 20)
            {

                double initTimeTrajectory;
                initTimeTrajectory = m_time + m_newTrajectoryMergeCounter * m_dT;

                iDynTree::Transform measuredTransform = m_isLeftFixedFrame.front() ?
                    m_rightTrajectory[m_newTrajectoryMergeCounter] :
                    m_leftTrajectory[m_newTrajectoryMergeCounter];

                // ask for a new trajectory
                if(!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                       measuredTransform, m_newTrajectoryMergeCounter,
                                       m_desiredPosition))
                {
                    yError() << "[updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if(m_newTrajectoryMergeCounter == 2)
            {
                if(!updateTrajectories(m_newTrajectoryMergeCounter))
                {
                    yError() << "[updateModule] Error while updating trajectories. They were not computed yet.";
                    return false;
                }
                m_newTrajectoryRequired = false;
                resetTrajectory = true;
            }

            m_newTrajectoryMergeCounter--;
        }

        if (m_PIDHandler->usingGainScheduling())
        {
            if (!m_PIDHandler->updatePhases(m_leftInContact, m_rightInContact, m_time))
            {
                yError() << "[updateModule] Unable to get the update PID.";
                return false;
            }
        }

        // get feedbacks and evaluate useful quantities
        if(!getFeedbacks(100))
        {
            yError() << "[updateModule] Unable to get the feedback.";
            return false;
        }
        
        if(m_useHeadIMU || m_useFeetIMU)
        {
          checkWalkingStatus();
          if(m_walkingStatus == WalkingStatus::Unknown)
          {
            yError() << "[IMU] Unknown walking status!";
            return false;
          }
          
          if((m_useIMUDS && m_walkingStatus == WalkingStatus::DSStable) || !m_useIMUDS)
          {
            parseIMUData();
            if(m_updateOnceDS && !m_DSSwitchedOut && m_ortChanged)
              m_inertial_R_worldFrame = iDynTree::Rotation::Identity()*m_inertial_R_worldFrame;
            else
            {
              if(m_useIMUFiltering)
                updateInertiaRWorld(m_HeadIMUDataFilt,m_LFootIMUDataFilt,m_RFootIMUDataFilt,false);
              else
                updateInertiaRWorld(m_HeadIMUData,m_LFootIMUData,m_RFootIMUData,false);
            }
          }
          
          if(m_ortChangeIndex < m_inertial_R_worldFrame_vec.size())
          {
            m_inertial_R_worldFrame = m_inertial_R_worldFrame_vec[m_ortChangeIndex];
            m_ortChangeIndex++;
          }
        }

        if(!updateFKSolver())
        {
            yError() << "[updateModule] Unable to update the FK solver.";
            return false;
        }

        if(!evaluateCoM(measuredCoM, measuredCoMVelocity))
        {
            yError() << "[updateModule] Unable to evaluate the CoM.";
            return false;
        }

        if(!evaluateDCM(measuredDCM))
        {
            yError() << "[updateModule] Unable to evaluate the DCM.";
            return false;
        }

        if(!evaluateZMP(measuredZMP))
        {
            yError() << "[updateModule] Unable to evaluate the ZMP.";
            return false;
        }

        // evaluate 3D-LIPM reference signal
        m_stableDCMModel->setInput(m_DCMPositionDesired.front());
        if(!m_stableDCMModel->integrateModel())
        {
            yError() << "[updateModule] Unable to propagate the 3D-LIPM.";
            return false;
        }

        iDynTree::Vector2 desiredCoMPositionXY;
        if(!m_stableDCMModel->getCoMPosition(desiredCoMPositionXY))
        {
            yError() << "[updateModule] Unable to get the desired CoM position.";
            return false;
        }

        iDynTree::Vector2 desiredCoMVelocityXY;
        if(!m_stableDCMModel->getCoMVelocity(desiredCoMVelocityXY))
        {
            yError() << "[updateModule] Unable to get the desired CoM velocity.";
            return false;
        }

        // DCM controller
        iDynTree::Vector2 desiredZMP;
        if(m_useMPC)
        {
            if((m_useFeetIMU || m_useHeadIMU))
            {
              computeInclinationPlane();
              if(m_useBiasMethod)
              {
                iDynTree::Position currentGravity;
                currentGravity.zero();
                currentGravity(2) = 9.81;
                currentGravity = m_inertial_R_worldFrame*currentGravity;
                m_walkingController->setBias(currentGravity(0),currentGravity(1));
                updateOmega(currentGravity);
              }
              else
                updateOmega(m_desiredZMPX,m_desiredZMPY);
            }
            
            // Model predictive controller
            m_profiler->setInitTime("MPC");
            if(!m_walkingController->setConvexHullConstraint(m_leftTrajectory, m_rightTrajectory,
                                                             m_leftInContact, m_rightInContact))
            {
                yError() << "[updateModule] unable to evaluate the convex hull.";
                return false;
            }

            if(!m_walkingController->setFeedback(measuredDCM))
            {
                yError() << "[updateModule] unable to set the feedback.";
                return false;
            }

            if(!m_walkingController->setReferenceSignal(m_DCMPositionDesired, resetTrajectory))
            {
                yError() << "[updateModule] unable to set the reference Signal.";
                return false;
            }

            if(!m_walkingController->solve())
            {
                yError() << "[updateModule] Unable to solve the problem.";
                return false;
            }

            if(!m_walkingController->getControllerOutput(desiredZMP))
            {
                yError() << "[updateModule] Unable to get the MPC output.";
                return false;
            }
            
            m_desiredZMPX = desiredZMP(0);
            m_desiredZMPY = desiredZMP(1);
            m_profiler->setEndTime("MPC");
        }
        else
        {
            if((m_useFeetIMU || m_useHeadIMU))
            {
              computeInclinationPlane();
              if(m_useBiasMethod)
              {
                iDynTree::Position currentGravity;
                currentGravity.zero();
                currentGravity(2) = 9.81;
                currentGravity = m_inertial_R_worldFrame*currentGravity;
                m_walkingDCMReactiveController->setBias(currentGravity(0),currentGravity(1));
                updateOmega(currentGravity);
              }
              else
                updateOmega(m_desiredZMPX,m_desiredZMPY);
            }
            
            m_walkingDCMReactiveController->setFeedback(measuredDCM);
            m_walkingDCMReactiveController->setReferenceSignal(m_DCMPositionDesired.front(),
                                                               m_DCMVelocityDesired.front());

            if(!m_walkingDCMReactiveController->evaluateControl())
            {
                yError() << "[updateModule] Unable to evaluate the DCM control output.";
                return false;
            }

            if(!m_walkingDCMReactiveController->getControllerOutput(desiredZMP))
            {
                yError() << "[updateModule] Unable to get the DCM control output.";
                return false;
            }
            
            m_desiredZMPX = desiredZMP(0);
            m_desiredZMPY = desiredZMP(1);
        }

        // inner COM-ZMP controller
        // if the the norm of desired DCM velocity is lower than a threshold then the robot
        // is stopped
        double threshold = 0.001;
        bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
        m_walkingZMPController->setPhase(stancePhase || m_robotState == WalkingFSM::OnTheFly);

        // set feedback and the desired signal
        m_walkingZMPController->setFeedback(measuredZMP, measuredCoM);
        m_walkingZMPController->setReferenceSignal(desiredZMP, desiredCoMPositionXY,
                                                   desiredCoMVelocityXY);

        if(!m_walkingZMPController->evaluateControl())
        {
            yError() << "[updateModule] Unable to evaluate the ZMP control output.";
            return false;
        }

        iDynTree::Vector2 outputZMPCoMControllerPosition, outputZMPCoMControllerVelocity;
        if(!m_walkingZMPController->getControllerOutput(outputZMPCoMControllerPosition,
                                                        outputZMPCoMControllerVelocity))
        {
            yError() << "[updateModule] Unable to get the ZMP controller output.";
            return false;
        }

        // inverse kinematics
        m_profiler->setInitTime("IK");

        iDynTree::Position desiredCoMPosition;
        desiredCoMPosition(0) = outputZMPCoMControllerPosition(0);
        desiredCoMPosition(1) = outputZMPCoMControllerPosition(1);

        if(m_robotState == WalkingFSM::OnTheFly)
        {
            m_heightSmoother->computeNextValues(yarp::sig::Vector(1,m_comHeightTrajectory.front()));
            desiredCoMPosition(2) = m_heightSmoother->getPos()[0];
        }
        else
            desiredCoMPosition(2) = m_comHeightTrajectory.front();


        iDynTree::Vector3 desiredCoMVelocity;
        desiredCoMVelocity(0) = outputZMPCoMControllerVelocity(0);
        desiredCoMVelocity(1) = outputZMPCoMControllerVelocity(1);
        desiredCoMVelocity(2) = m_comHeightVelocity.front();

        // evaluate desired neck transformation
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        yawRotation = iDynTree::Rotation::RotZ(meanYaw);
        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(m_useQPIK && m_robotState != WalkingFSM::OnTheFly)
        {
            // integrate dq because velocity control mode seems not available
            yarp::sig::Vector bufferVelocity(m_actuatedDOFs);
            yarp::sig::Vector bufferPosition(m_actuatedDOFs);

            if(!m_FKSolver->setInternalRobotState(m_qDesired, m_dqDesired_osqp))
            {
                yError() << "[updateFKSolver] Unable to evaluate the CoM.";
                return false;
            }

            if(m_useOSQP)
            {
                if(!solveQPIK(m_QPIKSolver_osqp, desiredCoMPosition,
                              desiredCoMVelocity, measuredCoM,
                              yawRotation, m_dqDesired_osqp))
                {
                    yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                    return false;
                }

                iDynTree::toYarp(m_dqDesired_osqp, bufferVelocity);
            }
            else
            {
                if(!solveQPIK(m_QPIKSolver_qpOASES, desiredCoMPosition,
                              desiredCoMVelocity, measuredCoM,
                              yawRotation, m_dqDesired_qpOASES))
                {
                    yError() << "[updateModule] Unable to solve the QP problem with osqp.";
                    return false;
                }

                iDynTree::toYarp(m_dqDesired_qpOASES, bufferVelocity);
            }


            bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
            iDynTree::toiDynTree(bufferPosition, m_qDesired);
        }
        else
        {
            if(m_robotState == WalkingFSM::OnTheFly)
            {
                iDynTree::VectorDynSize desiredJointInRad(m_actuatedDOFs);
                m_jointsSmoother->computeNextValues(m_desiredJointInRadYarp);
                iDynTree::toiDynTree(m_jointsSmoother->getPos(), desiredJointInRad);
                if (!m_IKSolver->setDesiredJointConfiguration(desiredJointInRad))
                {
                    yError() << "[updateModule] Unable to set the desired Joint Configuration.";
                    return false;
                }
            }

            if(m_IKSolver->usingAdditionalRotationTarget())
            {

                if(m_robotState == WalkingFSM::OnTheFly)
                {
                    m_additionalRotationWeightSmoother->computeNextValues(yarp::sig::Vector(1,
                                                                                            m_additionalRotationWeightDesired));
                    double rotationWeight = m_additionalRotationWeightSmoother->getPos()[0];
                    if (!m_IKSolver->setAdditionalRotationWeight(rotationWeight))
                    {
                        yError() << "[updateModule] Unable to set the additional rotational weight.";
                        return false;
                    }

                    m_desiredJointWeightSmoother->computeNextValues(yarp::sig::Vector(1, m_desiredJointsWeight));
                    double jointWeight = m_desiredJointWeightSmoother->getPos()[0];
                    if (!m_IKSolver->setDesiredJointsWeight(jointWeight))
                    {
                        yError() << "[updateModule] Unable to set the desired joint weight.";
                        return false;
                    }
                }

                if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
                {
                    yError() << "[updateModule] Error updating the inertia to world frame rotation.";
                    return false;
                }

                if(!m_IKSolver->setFullModelFeedBack(m_positionFeedbackInRadians))
                {
                    yError() << "[updateModule] Error while setting the feedback to the inverse Kinematics.";
                    return false;
                }

                if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                          desiredCoMPosition, m_qDesired))
                {
                    yError() << "[updateModule] Error during the inverse Kinematics iteration.";
                    return false;
                }
            }
        }
        m_profiler->setEndTime("IK");

        if(m_useQPIK)
        {
            if(!setDirectPositionReferences(m_qDesired))
            {
                yError() << "[updateModule] Error while setting the reference position to iCub.";
                return false;
            }
        }
        else
        {
            if(!setDirectPositionReferences(m_qDesired))
            {
                yError() << "[updateModule] Error while setting the reference position to iCub.";
                return false;
            }
        }

        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();

        iDynTree::VectorDynSize errorL(6), errorR(6);
        if(m_robotState != WalkingFSM::OnTheFly && m_useQPIK)
        {
            if(m_useOSQP)
            {
                m_QPIKSolver_osqp->getRightFootError(errorR);
                m_QPIKSolver_osqp->getLeftFootError(errorL);
            }
            else
            {
                m_QPIKSolver_qpOASES->getRightFootError(errorR);
                m_QPIKSolver_qpOASES->getLeftFootError(errorL);
            }
        }

        // send data to the WalkingLogger
        if(m_dumpData)
        {
            auto leftFoot = m_FKSolver->getLeftFootToWorldTransform();
            auto rightFoot = m_FKSolver->getRightFootToWorldTransform();
            m_walkingLogger->sendData(measuredDCM, m_DCMPositionDesired.front(), m_DCMVelocityDesired.front(),
                                      measuredZMP, desiredZMP, measuredCoM,
                                      desiredCoMPositionXY, desiredCoMVelocityXY,
                                      leftFoot.getPosition(), leftFoot.getRotation().asRPY(),
                                      rightFoot.getPosition(), rightFoot.getRotation().asRPY(),
                                      m_leftTrajectory.front().getPosition(), m_leftTrajectory.front().getRotation().asRPY(),
                                      m_rightTrajectory.front().getPosition(), m_rightTrajectory.front().getRotation().asRPY(),
                                      errorL, errorR);

            // m_walkingLogger->sendData(m_dqDesired_osqp, m_dqDesired_qpOASES);
        }

        propagateTime();

        if(m_robotState != WalkingFSM::OnTheFly)
            // propagate all the signals
            propagateReferenceSignals();

        if((m_robotState == WalkingFSM::OnTheFly) && (m_time > m_onTheFlySmoothingTime))
        {
            // reset gains and desired joint position
            iDynTree::VectorDynSize desiredJointInRad(m_actuatedDOFs);
            iDynTree::toiDynTree(m_desiredJointInRadYarp, desiredJointInRad);
            if (!m_IKSolver->setDesiredJointConfiguration(desiredJointInRad))
            {
                yError() << "[updateModule] Unable to set the desired Joint Configuration.";
                return false;
            }

            if (!m_IKSolver->setAdditionalRotationWeight(m_additionalRotationWeightDesired))
            {
                yError() << "[updateModule] Unable to set the additional rotational weight.";
                return false;
            }

            if (!m_IKSolver->setDesiredJointsWeight(m_desiredJointsWeight))
            {
                yError() << "[updateModule] Unable to set the desired joint weight.";
                return false;
            }
            m_robotState = WalkingFSM::Stance;
            m_firstStep = true;

            // reset time
            m_time = 0.0;

            yarp::sig::Vector buffer(m_qDesired.size());
            iDynTree::toYarp(m_qDesired, buffer);
            // instantiate Integrator object
            m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);
        }
        else if(m_firstStep)
            m_firstStep = false;

    }
    return true;
}

bool WalkingModule::getFeedbacks(unsigned int maxAttempts)
{
    if(!m_encodersInterface)
    {
        yError() << "[getFeedbacks] Encoders I/F is not ready";
        return false;
    }

    bool okPosition = false;
    bool okVelocity = false;

    bool okLeftWrench = false;
    bool okRightWrench = false;

    unsigned int attempt = 0;

    do
    {
        if(!okPosition)
            okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

        if(!okVelocity)
            okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackInDegrees.data());

        if(!okLeftWrench)
        {
            yarp::sig::Vector *leftWrenchRaw = NULL;
            leftWrenchRaw = m_leftWrenchPort.read(false);
            if(leftWrenchRaw != NULL)
            {
                m_leftWrenchInput = *leftWrenchRaw;
                okLeftWrench = true;
            }
        }

        if(!okRightWrench)
        {
            yarp::sig::Vector *rightWrenchRaw = NULL;
            rightWrenchRaw = m_rightWrenchPort.read(false);
            if(rightWrenchRaw != NULL)
            {
                m_rightWrenchInput = *rightWrenchRaw;
                okRightWrench = true;
            }
        }

        if(okVelocity && okPosition && okLeftWrench && okRightWrench)
        {
            if(m_useVelocityFilter)
            {
                // filter the joint position and the velocity
                m_velocityFeedbackInDegreesFiltered = m_velocityFilter->filt(m_velocityFeedbackInDegrees);
                for(unsigned j = 0; j < m_actuatedDOFs; ++j)
                {
                    m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));
                    m_velocityFeedbackInRadians(j) = iDynTree::deg2rad(m_velocityFeedbackInDegreesFiltered(j));
                }
            }
            else
            {
                for(unsigned j = 0; j < m_actuatedDOFs; ++j)
                {
                    m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));
                    m_velocityFeedbackInRadians(j) = iDynTree::deg2rad(m_velocityFeedbackInDegrees(j));
                }
            }
            if(m_useWrenchFilter)
            {
                if(m_firstStep)
                {
                    m_leftWrenchFilter->init(m_leftWrenchInput);
                    m_rightWrenchFilter->init(m_rightWrenchInput);
                }
                m_leftWrenchInputFiltered = m_leftWrenchFilter->filt(m_leftWrenchInput);
                m_rightWrenchInputFiltered = m_rightWrenchFilter->filt(m_rightWrenchInput);

                if(!iDynTree::toiDynTree(m_leftWrenchInputFiltered, m_leftWrench))
                {
                    yError() << "[getFeedbacks] Unable to convert left foot wrench.";
                    return false;
                }
                if(!iDynTree::toiDynTree(m_rightWrenchInputFiltered, m_rightWrench))
                {
                    yError() << "[getFeedbacks] Unable to convert right foot wrench.";
                    return false;
                }
            }
            else
            {
                if(!iDynTree::toiDynTree(m_leftWrenchInput, m_leftWrench))
                {
                    yError() << "[getFeedbacks] Unable to convert left foot wrench.";
                    return false;
                }
                if(!iDynTree::toiDynTree(m_rightWrenchInput, m_rightWrench))
                {
                    yError() << "[getFeedbacks] Unable to convert right foot wrench.";
                    return false;
                }
            }
            return true;
        }

        yarp::os::Time::delay(0.001);
        attempt++;
    } while(attempt < maxAttempts);

    yInfo() << "[getFeedbacks] The following readings failed:";
    if(!okPosition)
        yInfo() << "\t - Position encoders";

    if(!okVelocity)
        yInfo() << "\t - Velocity encoders";

    if(!okLeftWrench)
        yInfo() << "\t - Left wrench";

    if(!okRightWrench)
        yInfo() << "\t - Right wrench";

    return false;
}

bool WalkingModule::evaluateZMP(iDynTree::Vector2& zmp)
{
    if(m_FKSolver == nullptr)
    {
        yError() << "[evaluateZMP] The FK solver is not ready.";
        return false;
    }

    iDynTree::Position zmpLeft, zmpRight, zmpWorld;
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;

    if(m_rightWrench.getLinearVec3()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -m_rightWrench.getAngularVec3()(1) / m_rightWrench.getLinearVec3()(2);
        zmpRight(1) = m_rightWrench.getAngularVec3()(0) / m_rightWrench.getLinearVec3()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
    }

    if(m_leftWrench.getLinearVec3()(2) < 0.001)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -m_leftWrench.getAngularVec3()(1) / m_leftWrench.getLinearVec3()(2);
        zmpLeft(1) = m_leftWrench.getAngularVec3()(0) / m_leftWrench.getLinearVec3()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
    }

    double totalZ = m_rightWrench.getLinearVec3()(2) + m_leftWrench.getLinearVec3()(2);
    if(totalZ < 0.1)
    {
        yError() << "[evaluateZMP] The total z-component of contact wrenches is too low.";
        return false;
    }

    zmpLeft = m_FKSolver->getLeftFootToWorldTransform() * zmpLeft;
    zmpRight = m_FKSolver->getRightFootToWorldTransform() * zmpRight;

    // the global zmp is given by a weighted average
    iDynTree::toEigen(zmpWorld) = ((m_leftWrench.getLinearVec3()(2) * zmpLeftDefined) / totalZ)
        * iDynTree::toEigen(zmpLeft) +
        ((m_rightWrench.getLinearVec3()(2) * zmpRightDefined)/totalZ) * iDynTree::toEigen(zmpRight);

    zmp(0) = zmpWorld(0);
    zmp(1) = zmpWorld(1);

    return true;
}

bool WalkingModule::getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                  std::pair<int, double>& worstError)
{
    if(!m_encodersInterface)
    {
        yError() << "[getWorstError] The encoder I/F is not ready";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()))
    {
        yError() << "[getWorstError] Error reading encoders.";
        return false;
    }

    // clear the std::pair
    worstError.first = -1;
    worstError.second = 0.0;
    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackInDegrees[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        if(absoluteJointErrorRad > worstError.second)
        {
            worstError.first = i;
            worstError.second = absoluteJointErrorRad;
        }
    }
    return true;
}

bool WalkingModule::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if(!m_controlModeInterface)
    {
        yError() << "[switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if(!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError() << "[switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool WalkingModule::setPositionReferences(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                          const double& positioningTimeSec)
{
    if(m_positionInterface == nullptr)
    {
        yError() << "[setPositionReferences] Position I/F is not ready.";
        return false;
    }

    std::pair<int, double> worstErrorRad(-1, 0.0);

    if(!getWorstError(desiredJointPositionsRad, worstErrorRad))
    {
        yError() << "[setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstErrorRad.second < 0.03)
        return true;

    if(positioningTimeSec < 0.01)
    {
        yError() << "[setPositionReferences] The positioning time is too short.";
        return false;
    }

    // switch control mode
    if(!switchToControlMode(VOCAB_CM_POSITION))
    {
        yError() << "[setPositionReferences] Failed in setting POSITION mode.";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()))
    {
        yError() << "[setPositionReferences] Error while reading encoders.";
        return false;
    }

    worstErrorRad.first = -1;
    worstErrorRad.second = 0.0;
    std::vector<double> refSpeeds(m_actuatedDOFs);

    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackInDegrees[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        refSpeeds[i] = std::max(3.0, iDynTree::rad2deg(absoluteJointErrorRad) / positioningTimeSec);
    }

    if(!m_positionInterface->setRefSpeeds(refSpeeds.data()))
    {
        yError() << "[setPositionReferences] Error while setting the desired speed of joints.";
        return false;
    }

    // convert a radians vector into a degree vector
    iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(desiredJointPositionsRad) * iDynTree::rad2deg(1);

    if(!m_positionInterface->positionMove(m_toDegBuffer.data()))
    {
        yError() << "Error while setting the desired positions.";
        return false;
    }

    bool terminated = false;
    int attempt = 0;
    do {
        if(!terminated)
            m_positionInterface->checkMotionDone(& terminated);

        if(terminated)
        {
            if(!getWorstError(desiredJointPositionsRad, worstErrorRad))
            {
                yError() << "[setPositionReferences] Unable to get the worst error.";
                return false;
            }

            if(worstErrorRad.second < 2.0)
                return true;
        }

        yarp::os::Time::delay(positioningTimeSec * 0.5);
        attempt++;
    } while(attempt < 4);

    if(terminated)
        yError() << "The joint " << m_axesList[worstErrorRad.first]
                 << " was the worst in positioning with an error of "
                 << worstErrorRad.second << "rad.";
    else
        yError() << "Unable to complete the motion.";

    return false;
}

bool WalkingModule::setDirectPositionReferences(const iDynTree::VectorDynSize& desiredPositionsRad)
{
    if(m_positionDirectInterface == nullptr)
    {
        yError() << "[setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[setDirectPositionReferences] Encoders I/F not ready.";
        return false;
    }

    if(desiredPositionsRad.size() != m_actuatedDOFs)
    {
        yError() << "[setDirectPositionReferences] Dimension mismatch between desired position "
                 << "vector and the number of controlled joints.";
        return false;
    }

    std::pair<int, double> worstErrorRad(-1, 0.0);

    if(!getWorstError(desiredPositionsRad, worstErrorRad))
    {
        yError() << "[setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstErrorRad.second > 0.5)
    {
        yError() << "[setDirectPositionReferences] The worst error between the current and the "
                 << "desired position of the " << worstErrorRad.first
                 << "-th joint is greater than 0.5 rad.";
        return false;
    }

    iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(desiredPositionsRad) * iDynTree::rad2deg(1);

    if(!m_positionDirectInterface->setPositions(m_toDegBuffer.data()))
    {
        yError() << "[setDirectPositionReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool WalkingModule::setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad)
{
    if(m_velocityInterface == nullptr)
    {
        yError() << "[setVelocityReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[setVelocityReferences] Encoders I/F not ready.";
        return false;
    }

    if(desiredVelocityRad.size() != m_actuatedDOFs)
    {
        yError() << "[setVelocityReferences] Dimension mismatch between desired velocity "
                 << "vector and the number of controlled joints.";
        return false;
    }

    if((iDynTree::toEigen(m_toDegBuffer).minCoeff() < -0.25) ||
       (iDynTree::toEigen(m_toDegBuffer).minCoeff() > 0.25))
    {
        yError() << "[setVelocityReferences] The absolute value of the desired velocity is higher than 0.25 rad/s.";
        return false;
    }

    iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(desiredVelocityRad) * iDynTree::rad2deg(1);

    if(!m_velocityInterface->velocityMove(m_toDegBuffer.data()))
    {
        yError() << "[setVelocityReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool WalkingModule::prepareRobot(bool onTheFly)
{
    if(m_robotState != WalkingFSM::Configured)
    {
        yError() << "[prepareRobot] You cannot prepare the robot again.";
        return false;
    }

    iDynTree::Position measuredCoM;
    iDynTree::Vector3 measuredCoMVelocity;
    iDynTree::Transform leftToRightTransform;

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    if(!getFeedbacks(10))
    {
        yError() << "[onTheFlyStartWalking] Unable to get the feedback.";
        return false;
    }

    if(onTheFly || m_useHeadIMU || m_useFeetIMU)
    {
        if(!m_FKSolver->setBaseOnTheFly())
	{
           yError() << "[onTheFlyStartWalking] Unable to set the onTheFly base.";
	   return false;
	}

	if(!m_FKSolver->setInternalRobotState(m_positionFeedbackInRadians, m_velocityFeedbackInRadians))
	{
	  yError() << "[onTheFlyStartWalking] Unable to evaluate the CoM.";
	  return false;
	}

	// evaluate the left to right transformation, the inertial frame is on the left foot
	leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

	// evaluate the first trajectory. The robot does not move!
	if(!generateFirstTrajectories(leftToRightTransform))
	{
	   yError() << "[onTheFlyStartWalking] Failed to evaluate the first trajectories.";
	   return false;
	}
	
	// set up the IMU
	if(m_useHeadIMU || m_useFeetIMU)
        {
          checkWalkingStatus();
          if(m_walkingStatus == WalkingStatus::Unknown)
          {
            yError() << "[IMU] Unknown walking status!";
            return false;
          }
          
          parseIMUData();
          if(m_useHeadIMU)
          {
            if(m_useIMUFiltering)
              computeEarthToWorldHead(m_HeadIMUDataFilt);
            else
              computeEarthToWorldHead(m_HeadIMUData);
          }
          if(m_useFeetIMU)
          {
            if(m_useIMUFiltering)
              computeEarthToWorld(m_LFootIMUDataFilt,m_RFootIMUDataFilt);
            else
              computeEarthToWorld(m_LFootIMUData,m_RFootIMUData);
          }
          if(m_useIMUFiltering)
            updateInertiaRWorld(m_HeadIMUDataFilt,m_LFootIMUDataFilt,m_RFootIMUDataFilt,true);
          else
            updateInertiaRWorld(m_HeadIMUData,m_LFootIMUData,m_RFootIMUData,true);
          m_DSSwitchedOut = false;
          
          m_nominalWorldRoll = m_inertial_R_worldFrame.asRPY()(0);
          m_nominalWorldPitch = m_inertial_R_worldFrame.asRPY()(1);
          
          computeInclinationPlane();
          if(m_useBiasMethod)
          {
            iDynTree::Position currentGravity;
            currentGravity.zero();
            currentGravity(2) = 9.81;
            currentGravity = m_inertial_R_worldFrame*currentGravity;
            if(m_useMPC)
              m_walkingController->setBias(currentGravity(0),currentGravity(1));
            else
              m_walkingDCMReactiveController->setBias(currentGravity(0),currentGravity(1));
            updateOmega(currentGravity);
          }
          else
            updateOmega(m_desiredZMPX,m_desiredZMPY);
        }
    }
    else
    {
        // evaluate the first trajectory. The robot does not move! So the first trajectory
        if(!generateFirstTrajectories())
        {
	    yError() << "[prepareRobot] Failed to evaluate the first trajectories.";
	    return false;
        }
    }

    // reset the gains
    if (m_PIDHandler->usingGainScheduling())
    {
        if (!(m_PIDHandler->reset()))
            return false;
    }

    for(unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));

    if(!m_IKSolver->setFullModelFeedBack(m_positionFeedbackInRadians))
    {
        yError() << "[prepareRobot] Error while setting the feedback to the IK solver.";
        return false;
    }

    iDynTree::Position desiredCoMPosition;
    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0);
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();

    if(m_IKSolver->usingAdditionalRotationTarget())
    {
        // get the yow angle of both feet
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        // evaluate the mean of the angles
        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        // it is important to notice that the inertial frames rotate with the robot
        yawRotation = iDynTree::Rotation::RotZ(meanYaw);

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }

    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                              desiredCoMPosition, m_qDesired))
    {
        yError() << "[prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }

    if(!setPositionReferences(m_qDesired, 5.0))
    {
        yError() << "[prepareRobot] Error while setting the initial position.";
        return false;
    }

    if(m_useQPIK)
    {
        if(!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
        {
            yError() << "[prepareRobot] Failed in setting POSITION DIRECT mode.";
            return false;
        }
    }
    else
    {
        if(!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
        {
            yError() << "[prepareRobot] Failed in setting POSITION DIRECT mode.";
            return false;
        }
    }

    // send the reference again in order to reduce error
    if(!setDirectPositionReferences(m_qDesired))
    {
        yError() << "[prepareRobot] Error while setting the initial position using "
                 << "POSITION DIRECT mode.";
        return false;
    }

    yarp::sig::Vector buffer(m_qDesired.size());
    iDynTree::toYarp(m_qDesired, buffer);
    // instantiate Integrator object
    m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);

    // reset the models
    m_walkingZMPController->reset(m_DCMPositionDesired.front());
    m_stableDCMModel->reset(m_DCMPositionDesired.front());

    m_robotState = WalkingFSM::Prepared;
    return true;
}

bool WalkingModule::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::generateFirstTrajectories()
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories())
    {
        yError() << "[generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
    {
        yError() << "[generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                       const iDynTree::Transform& measuredTransform,
                                       const size_t& mergePoint, const iDynTree::Vector2& desiredPosition)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if(mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }

    yInfo() << "init Time before updateTrajectories " << initTime;

    if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                  m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                  measuredTransform, desiredPosition))
    {
        yError() << "[askNewTrajectories] Unable to update the trajectory.";
        return false;
    }
    return true;
}

bool WalkingModule::updateTrajectories(const size_t& mergePoint)
{
    if(!(m_trajectoryGenerator->isTrajectoryComputed()))
    {
        yError() << "[updateTrajectories] The trajectory is not computed.";
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;

    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // append vectors to deques
    StdHelper::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);

    StdHelper::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    StdHelper::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    StdHelper::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    m_mergePoints.pop_front();

    return true;
}

bool WalkingModule::updateFKSolver()
{
    // if(m_firstStep)
    // {
    //     if(!m_FKSolver->evaluateFirstWorldToBaseTransformation(m_leftTrajectory.front()))
    //     {
    //         yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
    //         return false;
    //     }
    // }
    // else
    // {
    //     if(!m_FKSolver->evaluateWorldToBaseTransformation(m_isLeftFixedFrame.front()))
    //     {
    //         yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
    //         return false;
    //     }
    // }

    if(!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                      m_rightTrajectory.front(),
                                                      m_isLeftFixedFrame.front()))
    {
        yError() << "[updateFKSolver] Unable to evaluate the world to base transformation.";
        return false;
    }

    if(!m_FKSolver->setInternalRobotState(m_positionFeedbackInRadians, m_velocityFeedbackInRadians))
    {
        yError() << "[updateFKSolver] Unable to evaluate the CoM.";
        return false;
    }

    return true;
}

bool WalkingModule::evaluateCoM(iDynTree::Position& comPosition, iDynTree::Vector3& comVelocity)
{
    if(m_FKSolver == nullptr)
    {
        yError() << "[evaluateCoM] The FK solver is not ready.";
        return false;
    }

    if(!m_FKSolver->evaluateCoM())
    {
        yError() << "[evaluateCoM] Unable to evaluate the CoM.";
        return false;
    }

    if(!m_FKSolver->getCoMPosition(comPosition))
    {
        yError() << "[evaluateCoM] Unable to get the CoM position.";
        return false;
    }

    if(!m_FKSolver->getCoMVelocity(comVelocity))
    {
        yError() << "[evaluateCoM] Unable to get the CoM velocity.";
        return false;
    }

    return true;
}

bool WalkingModule::evaluateDCM(iDynTree::Vector2& dcm)
{
    if(m_FKSolver == nullptr)
    {
        yError() << "[evaluateDCM] The FK solver is not ready.";
        return false;
    }

    if(!m_FKSolver->evaluateDCM())
    {
        yError() << "[evaluateDCM] Unable to evaluate the DCM.";
        return false;
    }

    if(!m_FKSolver->getDCM(dcm))
    {
        yError() << "[evaluateDCM] Unable to get the DCM.";
        return false;
    }

    return true;
}

bool WalkingModule::startWalking()
{
    if(m_robotState != WalkingFSM::Prepared)
    {
        yError() << "[startWalking] Unable to start walking if the robot is not prepared.";
        return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record","dcm_x", "dcm_y",
                    "dcm_des_x", "dcm_des_y",
                    "dcm_des_dx", "dcm_des_dy",
                    "zmp_x", "zmp_y",
                    "zmp_des_x", "zmp_des_y",
                    "com_x", "com_y", "com_z",
                    "com_des_x", "com_des_y",
                    "com_des_dx", "com_des_dy",
                    "lf_x", "lf_y", "lf_z",
                    "lf_roll", "lf_pitch", "lf_yaw",
                    "rf_x", "rf_y", "rf_z",
                    "rf_roll", "rf_pitch", "rf_yaw",
                    "lf_des_x", "lf_des_y", "lf_des_z",
                    "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                    "rf_des_x", "rf_des_y", "rf_des_z",
                    "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                    "lf_err_x", "lf_err_y", "lf_err_z",
                    "lf_err_roll", "lf_err_pitch", "lf_err_yaw",
                    "rf_err_x", "rf_err_y", "rf_err_z",
                    "rf_err_roll", "rf_err_pitch", "rf_err_yaw"});
        // "torso_pitch", "torso_roll", "torso_yaw",
        // "l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow",
        // "r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow",
        // "l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll",
        // "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"

    // m_walkingLogger->startRecord({"record",
    //             "torso_pitch_osqp", "torso_roll_osqp", "torso_yaw_osqp",
    //             "l_shoulder_pitch_osqp", "l_shoulder_roll_osqp", "l_shoulder_yaw_osqp", "l_elbow_osqp",
    //             "r_shoulder_pitch_osqp", "r_shoulder_roll_osqp", "r_shoulder_yaw_osqp", "r_elbow_osqp",
    //             "l_hip_pitch_osqp", "l_hip_roll_osqp", "l_hip_yaw_osqp", "l_knee_osqp", "l_ankle_pitch_osqp", "l_ankle_roll_osqp",
    //             "r_hip_pitch_osqp", "r_hip_roll_osqp", "r_hip_yaw_osqp", "r_knee_osqp", "r_ankle_pitch_osqp", "r_ankle_roll_osqp",
    //             "torso_pitch_qpOASES", "torso_roll_qpOASES", "torso_yaw_qpOASES",
    //             "l_shoulder_pitch_qpOASES", "l_shoulder_roll_qpOASES", "l_shoulder_yaw_qpOASES", "l_elbow_qpOASES",
    //             "r_shoulder_pitch_qpOASES", "r_shoulder_roll_qpOASES", "r_shoulder_yaw_qpOASES", "r_elbow_qpOASES",
    //             "l_hip_pitch_qpOASES", "l_hip_roll_qpOASES", "l_hip_yaw_qpOASES", "l_knee_qpOASES", "l_ankle_pitch_qpOASES", "l_ankle_roll_qpOASES",
    //             "r_hip_pitch_qpOASES", "r_hip_roll_qpOASES", "r_hip_yaw_qpOASES", "r_knee_qpOASES", "r_ankle_pitch_qpOASES", "r_ankle_roll_qpOASES"});
    }
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_robotState = WalkingFSM::Stance;
        m_firstStep = true;
    }

    return true;
}

bool WalkingModule::setGoal(double x, double y)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking && m_robotState != WalkingFSM::Stance)
        return false;

    if(x == 0 && y == 0 && m_robotState == WalkingFSM::Stance)
        return true;

    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if(m_mergePoints.empty())
    {
        if(!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[setGoal] The trajectory has already finished but the system is not in double support.";
            return false;
        }

        if(m_newTrajectoryRequired)
              return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged after x cycles
        m_newTrajectoryMergeCounter = 20;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        if(m_mergePoints.front() > 20)
            m_newTrajectoryMergeCounter = m_mergePoints.front();
        else if(m_mergePoints.size() > 1)
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = m_mergePoints[1];
        }
        else
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = 20;
        }
    }

    if(x == 0 && y == 0)
    {
        m_robotState = WalkingFSM::Stance;
        m_trajectoryGenerator->addTerminalStep(false);
    }
    else
    {
        m_robotState = WalkingFSM::Walking;
        m_trajectoryGenerator->addTerminalStep(true);
    }

    m_desiredPosition(0) = x;
    m_desiredPosition(1) = y;

    m_newTrajectoryRequired = true;

    return true;
}

bool WalkingModule::onTheFlyStartWalking(const double smoothingTime)
{
    if(m_robotState != WalkingFSM::Configured)
    {
        yError() << "[prepareRobot] You cannot prepare the robot again.";
        return false;
    }

    std::lock_guard<std::mutex> guard(m_mutex);

    iDynTree::Position measuredCoM;
    iDynTree::Vector3 measuredCoMVelocity;
    iDynTree::Transform leftToRightTransform;

    if(smoothingTime < 0.01)
    {
        yError() << "[onTheFlyStartWalking] The smoothing time seems too short.";
        return false;
    }

    m_onTheFlySmoothingTime = smoothingTime;

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    if(!getFeedbacks(10))
    {
        yError() << "[onTheFlyStartWalking] Unable to get the feedback.";
        return false;
    }

    if(!m_FKSolver->setBaseOnTheFly())
    {
        yError() << "[onTheFlyStartWalking] Unable to set the onTheFly base.";
        return false;
    }

    if(!m_FKSolver->setInternalRobotState(m_positionFeedbackInRadians, m_velocityFeedbackInRadians))
    {
        yError() << "[onTheFlyStartWalking] Unable to evaluate the CoM.";
        return false;
    }

    // evaluate the left to right transformation, the inertial frame is on the left foot
    leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

    // evaluate the first trajectory. The robot does not move!
    if(!generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[onTheFlyStartWalking] Failed to evaluate the first trajectories.";
        return false;
    }

    if(!updateFKSolver())
    {
        yError() << "[onTheFlyStartWalking] Unable to update the FK solver.";
        return false;
    }

    if(!evaluateCoM(measuredCoM, measuredCoMVelocity))
    {
        yError() << "[onTheFlyStartWalking] Unable to evaluate the CoM position and velocity.";
        return false;
    }

    // convert the initial position of the joint from iDynTree to YARP
    yarp::sig::Vector initialJointValuesInRadYarp;
    initialJointValuesInRadYarp.resize(m_actuatedDOFs);
    iDynTree::toYarp(m_positionFeedbackInRadians, initialJointValuesInRadYarp);

    m_jointsSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(initialJointValuesInRadYarp.size(),
                                                                    m_dT, smoothingTime / 1.5);
    m_heightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, m_dT, smoothingTime / 1.5);
    m_additionalRotationWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, m_dT,
                                                                                      smoothingTime);
    m_desiredJointWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, m_dT,
                                                                                smoothingTime);

    // set the initial position of the trajectories (desired quantities and gains)
    m_jointsSmoother->init(initialJointValuesInRadYarp);
    m_heightSmoother->init(yarp::sig::Vector(1, measuredCoM(2)));

    // the initial weight of the rotation matrix is 0
    // Magic trick!
    m_additionalRotationWeightSmoother->init(yarp::sig::Vector(1, 0.0));
    m_additionalRotationWeightDesired = m_IKSolver->additionalRotationWeight();

    // the initial weight for the joints position is 10 times the desired weight related to rotation
    // Magic trick!
    m_desiredJointWeightSmoother->init(yarp::sig::Vector(1, 10 * m_additionalRotationWeightDesired));
    m_desiredJointsWeight = m_IKSolver->desiredJointWeight();

    m_desiredJointInRadYarp.resize(m_actuatedDOFs);
    iDynTree::toYarp(m_IKSolver->desiredJointConfiguration(), m_desiredJointInRadYarp);

    if(!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
    {
        yError() << "[onTheFlyStartWalking] Failed in setting POSITION DIRECT mode.";
        return false;
    }

    // reset the models
    iDynTree::Vector2 buff;
    buff(0) = measuredCoM(0);
    buff(1) = measuredCoM(1);
    m_walkingZMPController->reset(buff);
    m_stableDCMModel->reset(buff);

    // reset the gains
    if (m_PIDHandler->usingGainScheduling())
    {
        if (!(m_PIDHandler->reset()))
            return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record","dcm_x", "dcm_y",
                    "dcm_des_x", "dcm_des_y",
                    "dcm_des_dx", "dcm_des_dy",
                    "zmp_x", "zmp_y",
                    "zmp_des_x", "zmp_des_y",
                    "com_x", "com_y", "com_z",
                    "com_des_x", "com_des_y",
                    "com_des_dx", "com_des_dy",
                    "lf_x", "lf_y", "lf_z",
                    "lf_roll", "lf_pitch", "lf_yaw",
                    "rf_x", "rf_y", "rf_z",
                    "rf_roll", "rf_pitch", "rf_yaw",
                    "lf_des_x", "lf_des_y", "lf_des_z",
                    "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                    "rf_des_x", "rf_des_y", "rf_des_z",
                    "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                    "lf_err_x", "lf_err_y", "lf_err_z",
                    "lf_err_roll", "lf_err_pitch", "lf_err_yaw",
                    "rf_err_x", "rf_err_y", "rf_err_z",
                    "rf_err_roll", "rf_err_pitch", "rf_err_yaw"});
    }

    m_robotState = WalkingFSM::OnTheFly;

    return true;
}

// This function is called only once at the beginning of the walking
bool WalkingModule::computeEarthToWorld(yarp::sig::Vector imudataL, yarp::sig::Vector imudataR)
{
  // when world is inclined
  iDynTree::Rotation worldOrt = iDynTree::Rotation::Identity();
  double worldPitch = 0;
  double worldRoll = 0;
  if(imudataL(0) > m_IMUWorldThresholdRoll || imudataL(1) > m_IMUWorldThresholdPitch || imudataR(0) > m_IMUWorldThresholdRoll || imudataR(1) > m_IMUWorldThresholdPitch )
  {
    if(std::abs(imudataL(0) - imudataR(0)) > m_IMUThresholdRoll)
      worldRoll = (imudataL(0) + imudataR(0))/2;
    else
      worldRoll = imudataL(0);
    
    if(std::abs(imudataL(1) - imudataR(1)) > m_IMUThresholdRoll)
      worldPitch = (imudataL(1) + imudataR(1))/2;
    else
      worldPitch = imudataL(1);
  }
  
  worldOrt = iDynTree::Rotation::RPY(worldRoll,worldPitch,0);
  
  iDynTree::Rotation lSoleToBase = worldOrt*m_FKSolver->getLeftFootToWorldTransform().getRotation();
  m_rotLFTToSole = m_FKSolver->getTransformBetweenFrames(m_IKSolver->getLeftFootFrame(),m_imuLFootFrame).getRotation();
  iDynTree::Rotation lIMUtoEarth = iDynTree::Rotation::RPY(imudataL(0),imudataL(1),imudataL(2));
  m_rotLEarthToWorld = lSoleToBase*m_rotLFTToSole*m_IMUToFT*lIMUtoEarth.inverse();
  
  iDynTree::Rotation rSoleToBase = worldOrt*m_FKSolver->getRightFootToWorldTransform().getRotation();
  m_rotRFTToSole = m_FKSolver->getTransformBetweenFrames(m_IKSolver->getRightFootFrame(),m_imuRFootFrame).getRotation();
  iDynTree::Rotation rIMUtoEarth = iDynTree::Rotation::RPY(imudataR(0),imudataR(1),imudataR(2));
  m_rotREarthToWorld = rSoleToBase*m_rotRFTToSole*m_IMUToFT*rIMUtoEarth.inverse();
}

bool WalkingModule::computeEarthToWorldHead(yarp::sig::Vector imudata)
{
  // when world is inclined
  iDynTree::Rotation worldOrt = iDynTree::Rotation::Identity();
  double worldPitch = 0;
  double worldRoll = 0;
  if(imudata(0) > m_IMUWorldThresholdRoll || imudata(1) > m_IMUWorldThresholdPitch)
  {
    worldRoll = imudata(0);
    worldPitch = imudata(1);
  }
  
  worldOrt = iDynTree::Rotation::RPY(worldRoll,worldPitch,0);
  iDynTree::Rotation headToBase = worldOrt*m_FKSolver->getFrameToWorldTransform(m_imuHeadFrame).getRotation();
  iDynTree::Rotation IMUtoEarth = iDynTree::Rotation::RPY(imudata(0),imudata(1),imudata(2));
  m_rotHeadEarthToWorld = headToBase*IMUtoEarth.inverse();
}

bool WalkingModule::computeFeetOrientation(yarp::sig::Vector imudataL, yarp::sig::Vector imudataR)
{  
  iDynTree::Rotation lIMUtoEarth = iDynTree::Rotation::RPY(imudataL(0),imudataL(1),imudataL(2));
  m_rotLFootIMU = m_rotLEarthToWorld*lIMUtoEarth*m_IMUToFT.inverse()*m_rotLFTToSole.inverse();
  
  iDynTree::Rotation rIMUtoEarth = iDynTree::Rotation::RPY(imudataR(0),imudataR(1),imudataR(2));
  m_rotRFootIMU = m_rotREarthToWorld*rIMUtoEarth*m_IMUToFT.inverse()*m_rotRFTToSole.inverse();
}

bool WalkingModule::computeHeadOrientation(yarp::sig::Vector imudata)
{  
  iDynTree::Rotation IMUtoEarth = iDynTree::Rotation::RPY(imudata(0),imudata(1),imudata(2));
  m_rotHeadIMU = m_rotHeadEarthToWorld*IMUtoEarth;
}

bool WalkingModule::updateInertiaRWorld(yarp::sig::Vector imudataHead, yarp::sig::Vector imudataL, yarp::sig::Vector imudataR, bool prepare)
{    
  // by default assume the feet to be on the same plane
  bool adaptOrt = false;
  iDynTree::Rotation inertiaRotMat = iDynTree::Rotation::Identity();
  
  if(m_useHeadIMU)
    computeHeadOrientation(imudataHead);
  if(m_useFeetIMU)
    computeFeetOrientation(imudataL,imudataR);
  
  iDynTree::Rotation headOrt;
  iDynTree::Vector3 diffHeadRot;
  iDynTree::Rotation rFootOrt;
  iDynTree::Rotation lFootOrt;
  iDynTree::Vector3 diffRFootRot;
  iDynTree::Vector3 diffLFootRot;
  diffHeadRot.zero();
  diffRFootRot.zero();
  diffLFootRot.zero();
  
  if(m_useHeadIMU)
  {
    headOrt = m_FKSolver->getFrameToWorldTransform(m_imuHeadFrame).getRotation();
    diffHeadRot = (headOrt.inverse()*m_rotHeadIMU).asRPY();
  }
  
  if(m_useFeetIMU)
  {    
    // check the threshold wrt the other stance foot
    if(m_prevWalkingStatus == WalkingStatus::RSS)
    {
      diffRFootRot = (m_rotRFootIMU.inverse()*m_rotRFootIMU).asRPY();
      diffLFootRot = (m_rotRFootIMU.inverse()*m_rotLFootIMU).asRPY();
    } else if(m_prevWalkingStatus == WalkingStatus::LSS)
    {
      diffRFootRot = (m_rotLFootIMU.inverse()*m_rotRFootIMU).asRPY();
      diffLFootRot = (m_rotLFootIMU.inverse()*m_rotLFootIMU).asRPY();
    }
  }
  
  if(m_useHeadIMU)
  {
    if(std::abs(diffHeadRot(0)) > m_IMUThresholdRoll)
      adaptOrt = true;
    if(std::abs(diffHeadRot(1)) > m_IMUThresholdPitch)
      adaptOrt = true;
  }
  
  if(m_useFeetIMU)
  {
    if(std::abs(diffRFootRot(0)) > m_IMUThresholdRoll)
      adaptOrt = true;
    if(std::abs(diffRFootRot(1)) > m_IMUThresholdPitch)
      adaptOrt = true;
    if(std::abs(diffLFootRot(0)) > m_IMUThresholdRoll)
      adaptOrt = true;
    if(std::abs(diffLFootRot(1)) > m_IMUThresholdPitch)
      adaptOrt = true;
  }
  
  if(m_useHeadIMU)
  {
    if(std::abs(m_rotHeadIMU.asRPY()(0) - m_nominalWorldRoll) > m_IMUWorldThresholdRoll || std::abs(m_rotHeadIMU.asRPY()(1) - m_nominalWorldPitch) > m_IMUWorldThresholdPitch)
      adaptOrt = true;
  }
  
  if(m_useFeetIMU)
  {
    if(std::abs(m_rotLFootIMU.asRPY()(0) - m_nominalWorldRoll) > m_IMUWorldThresholdRoll || std::abs(m_rotLFootIMU.asRPY()(1) - m_nominalWorldPitch) > m_IMUWorldThresholdPitch)
      adaptOrt = true;
    if(std::abs(m_rotRFootIMU.asRPY()(0) - m_nominalWorldRoll) > m_IMUWorldThresholdRoll || std::abs(m_rotRFootIMU.asRPY()(1) - m_nominalWorldPitch) > m_IMUWorldThresholdPitch)
      adaptOrt = true;
  }
  
  if(m_walkingStatus == WalkingStatus::DSStable)
  {
    std::cout << "Right foot IMU: " << iDynTree::rad2deg(imudataR(0)) << ", " << iDynTree::rad2deg(imudataR(1)) << std::endl;
    std::cout << "Left foot IMU: " << iDynTree::rad2deg(imudataR(0)) << ", " << iDynTree::rad2deg(imudataR(1)) << std::endl;
    std::cout << "Right foot: " << iDynTree::rad2deg(m_rotRFootIMU.asRPY()(0)) << ", " << iDynTree::rad2deg(m_rotRFootIMU.asRPY()(1)) << std::endl;
    std::cout << "Left foot: " << iDynTree::rad2deg(m_rotLFootIMU.asRPY()(0)) << ", " << iDynTree::rad2deg(m_rotLFootIMU.asRPY()(1)) << std::endl;
  }
  
  if(adaptOrt)
  {    
    double newRoll = m_inertial_R_worldFrame.asRPY()(0);
    double newPitch = m_inertial_R_worldFrame.asRPY()(1);
    
    if(m_useFeetIMU)
    {
      if(m_walkingStatus == WalkingStatus::DSStable)
      {
        // if feet not on the same plane, otherwise use the new touchdown foot
        if(std::abs(diffLFootRot(0) - diffRFootRot(0)) > m_IMUPlaneThreshold)
        {
          newRoll = (m_rotRFootIMU.asRPY()(0) + m_rotLFootIMU.asRPY()(0))/2;
        }
        else
        {
          if(m_prevWalkingStatus == WalkingStatus::RSS)
            newRoll = m_rotLFootIMU.asRPY()(0);
          else
            newRoll = m_rotRFootIMU.asRPY()(0);
        }
        if(std::abs(diffLFootRot(1) - diffRFootRot(1)) > m_IMUPlaneThreshold)
        {
          newPitch = (m_rotRFootIMU.asRPY()(1) + m_rotLFootIMU.asRPY()(1))/2;
        }
        else
        {
          if(m_prevWalkingStatus == WalkingStatus::RSS)
            newPitch = m_rotLFootIMU.asRPY()(1);
          else
            newPitch = m_rotRFootIMU.asRPY()(1);
        }
      }
      
      if(m_walkingStatus == WalkingStatus::RSS)
      {
        newRoll = m_rotRFootIMU.asRPY()(0);
      }
      if(m_walkingStatus == WalkingStatus::LSS)
      {
        newPitch = m_rotRFootIMU.asRPY()(1);
      }
    }
    
    if(m_useHeadIMU)
    {
      newRoll = m_rotHeadIMU.asRPY()(0);
      newPitch = m_rotHeadIMU.asRPY()(0);
    }
    
    inertiaRotMat = iDynTree::Rotation::RPY(-newRoll,-newPitch,m_inertial_R_worldFrame.asRPY()(2));
    
    std::cout << "!!!!New angles: " << iDynTree::rad2deg(newRoll) << ", " << iDynTree::rad2deg(newPitch) << ", " << 
    iDynTree::rad2deg(m_inertial_R_worldFrame.asRPY()(2)) << std::endl;
    std::cout << "!!!!Angles diff: " << diffHeadRot.toString() << ", " << diffLFootRot.toString() << ", " << 
    diffRFootRot.toString() << std::endl;
//     yInfo() << m_inertial_R_worldFrame.toString();
//     yInfo() << inertiaRotMat.toString();
  } else
    inertiaRotMat = m_inertial_R_worldFrame; // if does not adapt, then use previous rotation
    
  if(adaptOrt)
    yInfo() << ">>>>>>>>>>>>>>>!!! Changing ort !!!";
  m_inertial_R_worldFrame_new = inertiaRotMat;
  m_ortChanged = adaptOrt;
  
  if(prepare)
    m_inertial_R_worldFrame = m_inertial_R_worldFrame_new;
  else
  {
    if(m_ortChanged)
    {
      smoothOrtTransition(m_inertial_R_worldFrame.asRPY(),m_inertial_R_worldFrame_new.asRPY(),m_inertial_R_worldFrame_vec);
    }
  }
  
  return true;
}

bool WalkingModule::parseIMUData()
{
  // read IMU data  
  if(m_useHeadIMU)
  {
    yarp::sig::Vector* imuHeadInput = m_HeadIMUPort.read(false);
    if(imuHeadInput == NULL)
      return true;
    if(imuHeadInput->size() == 0)
    {
      yError() << "Could not read head IMU data properly.";
      return false;
    }
    
    for(int i = 0; i < m_HeadIMUData.size(); i++)
      m_HeadIMUData[i] = iDynTree::deg2rad(imuHeadInput->operator[](i));
    
    if(m_ignoreYaw)
      m_HeadIMUData[2] = 0;
  }
  
  if(m_useFeetIMU)
  {
    if(m_robot.compare("icubSim") == 0)
    {
      yarp::sig::Vector* imuRInput = m_RFootIMUPortSim.read(false);
      if(imuRInput == NULL)
        return true;
      yarp::sig::Vector* imuLInput = m_LFootIMUPortSim.read(false);
      if(imuLInput == NULL)
        return true;
      
      if(imuRInput->size() == 0)
      {
        yError() << "Could not read right foot IMU data properly.";
        return false;
      }
      if(imuLInput->size() == 0)
      {
        yError() << "Could not read left foot IMU data properly.";
        return false;
      }
      
      for(int i = 0; i < m_RFootIMUData.size(); i++)
      {
        m_RFootIMUData[i] = iDynTree::deg2rad(imuRInput->operator[](i));
        m_LFootIMUData[i] = iDynTree::deg2rad(imuLInput->operator[](i));
      }
      
      if(m_ignoreYaw)
      {
        m_RFootIMUData[2] = 0;
        m_LFootIMUData[2] = 0;
      }
    } 
    else
    {
      yarp::os::Bottle* imuRInput = m_RFootIMUPort.read(false)->get(3).asList()->get(0).asList()->get(0).asList();
      if(imuRInput == NULL)
        return true;
      yarp::os::Bottle* imuLInput = m_LFootIMUPort.read(false)->get(3).asList()->get(0).asList()->get(0).asList();
      if(imuLInput == NULL)
        return true;

      if(imuRInput->size() == 0)
      {
        yError() << "Could not read right foot IMU data properly.";
        return false;
      }
      if(imuLInput->size() == 0)
      {
        yError() << "Could not read left foot IMU data properly.";
        return false;
      }
      
      m_RFootIMUData[0] = iDynTree::deg2rad(-imuRInput->get(1).asDouble());
      m_RFootIMUData[1] = iDynTree::deg2rad(-imuRInput->get(2).asDouble());
      if(m_ignoreYaw)
        m_RFootIMUData[0] = 0;
      else
        m_RFootIMUData[2] = iDynTree::deg2rad(-imuRInput->get(0).asDouble());
      m_LFootIMUData[0] = iDynTree::deg2rad(-imuLInput->get(1).asDouble());
      m_LFootIMUData[1] = iDynTree::deg2rad(-imuLInput->get(2).asDouble());
      if(m_ignoreYaw)
        m_LFootIMUData[2] = 0;
      else
        m_LFootIMUData[2] = iDynTree::deg2rad(-imuLInput->get(0).asDouble());
    }
  }
  
  if(m_useIMUFiltering)
  {
    if(m_robotState == WalkingFSM::Configured)
    {
      if(m_useHeadIMU)
        m_HeadIMUFilter->init(m_HeadIMUData);
      if(m_useFeetIMU)
      {
        m_LFootIMUFilter->init(m_LFootIMUData);
        m_RFootIMUFilter->init(m_RFootIMUData);
      }
    }
    
    if(m_useHeadIMU)
      m_HeadIMUDataFilt = m_HeadIMUFilter->filt(m_HeadIMUData);
    if(m_useFeetIMU)
    {
      m_LFootIMUDataFilt = m_LFootIMUFilter->filt(m_LFootIMUData);
      m_RFootIMUDataFilt = m_RFootIMUFilter->filt(m_RFootIMUData);
    }
  }
  
  return true;
}

bool WalkingModule::checkWalkingStatus()
{
  // use 4 forces
  yarp::sig::Vector leftWrench(m_leftWrenchInput.size());
  yarp::sig::Vector rightWrench(m_rightWrenchInput.size());
  
  if(m_useWrenchFilter)
  {
    leftWrench = m_leftWrenchInputFiltered;
    rightWrench = m_rightWrenchInputFiltered;
  }
  else
  {
    leftWrench = m_leftWrenchInput;
    rightWrench = m_rightWrenchInput;
  }
  // check walking status according to wrench measurements if useFTDetection
  if(m_useFTDetection)
  {
    if(leftWrench(2) <= m_FTThreshold && rightWrench(2) <= m_FTThreshold)
        m_walkingStatus = WalkingStatus::Unknown;
    else if(leftWrench(2) > m_FTThreshold && rightWrench(2) > m_FTThreshold && !(m_walkingStatus == WalkingStatus::DSStable) && m_walkingStatus == WalkingStatus::DS)
    {
      if(checkFeetForces(leftWrench,rightWrench) || checkFeetVelocities())
      {
        m_walkingStatus = WalkingStatus::DSStable;
        m_ortChanged = false;
        m_DSSwitchedOut = false;
        yInfo() << "!! Change status to DSStable";
      }
    }
    else if(leftWrench(2) > m_FTThreshold && rightWrench(2) > m_FTThreshold && !(m_walkingStatus == WalkingStatus::DS || m_walkingStatus == WalkingStatus::DSStable))
    {
      m_walkingStatus = WalkingStatus::DS;
      yInfo() << "!! Change status to DS";
      m_ortChanged = false;
      m_DSSwitchedOut = true;
    }
    else if(leftWrench(2) > m_FTThreshold && rightWrench(2) <= m_FTThreshold && !(m_walkingStatus == WalkingStatus::LSS))
    {
        m_prevWalkingStatus = m_walkingStatus;
        m_walkingStatus = WalkingStatus::LSS;
        yInfo() << "!! Change status to LSS";
        m_ortChanged = false;
        m_DSSwitchedOut = true;
    }
    else if(leftWrench(2) <= m_FTThreshold && rightWrench(2) > m_FTThreshold && !(m_walkingStatus == WalkingStatus::RSS))
    {
      m_prevWalkingStatus = m_walkingStatus;
      m_walkingStatus = WalkingStatus::RSS;
      yInfo() << "!! Change status to RSS";
      m_ortChanged = false;
      m_DSSwitchedOut = true;
    }
  }
  else // use the planner
  {
    if(m_leftInContact.front() && m_rightInContact.front() && m_walkingStatus!=WalkingStatus::DS)
    {
      m_walkingStatus = WalkingStatus::DSStable;
      yInfo() << "!! Change status to DSStable";
      m_ortChanged = false;
      m_DSSwitchedOut = false;      
    }
    else if(m_leftInContact.front() && !m_rightInContact.front() && m_walkingStatus!=WalkingStatus::LSS)
    {
      m_prevWalkingStatus = m_walkingStatus;
      m_walkingStatus = WalkingStatus::LSS;
      yInfo() << "!! Change status to LSS";
      m_ortChanged = false;
      m_DSSwitchedOut = true;
    }
    else if(m_rightInContact.front() && !m_leftInContact.front() && m_walkingStatus!=WalkingStatus::RSS)
    {
      m_prevWalkingStatus = m_walkingStatus;
      m_walkingStatus = WalkingStatus::RSS;
      yInfo() << "!! Change status to RSS";
      m_ortChanged = false;
      m_DSSwitchedOut = true;
    }
  }
  
  return true;
}

void WalkingModule::computeFootForces(yarp::sig::Vector& wrench, yarp::sig::Vector& forces)
{
  // Foot dimension
  double d = 0.06;
  double L = 0.10;
  // compute the 4 forces of the foot
  double f1,f2,f3,f4;
  f1 = 0;
  f2 = 0;
  f3 = 0;
  f4 = 0;
  
  iDynTree::Vector2 CoP;
  CoP.zero();
  CoP(0) = -wrench(4)/wrench(2);
  CoP(1) = wrench(3)/wrench(2);
  
  f4 = 0.5*(std::max(0.0,-CoP(1)/d-CoP(0)/L)+std::min(0.5-CoP(1)/d,0.5-CoP(0)/L))*wrench(2);
  f1 = f4 + wrench(3)/d - wrench(4)/L;
  f2 = -f4 + wrench(2)/2 - wrench(3)/d;
  f3 = -f4 + wrench(2)/2 + wrench(4)/L;
  
  forces(0) = f1;
  forces(1) = f2;
  forces(2) = f3;
  forces(3) = f4;
}

bool WalkingModule::checkFeetForces(yarp::sig::Vector& leftWrench, yarp::sig::Vector& rightWrench)
{
  yarp::sig::Vector forcesL(4);
  yarp::sig::Vector forcesR(4);
  computeFootForces(leftWrench,forcesL);
  computeFootForces(rightWrench,forcesR);
//   std::cout << "Wrench left: " << leftWrench.toString() << std::endl;
//   std::cout << "Wrench right: " << rightWrench.toString() << std::endl;
//   std::cout << "Forces L foot: " << forcesL.toString() << std::endl;
//   std::cout << "Forces R foot: " << forcesR.toString() << std::endl;
  if(forcesL(0) > m_forcesThreshold && forcesL(1) > m_forcesThreshold && forcesL(2) > m_forcesThreshold && forcesL(3) > m_forcesThreshold && 
    forcesR(0) > m_forcesThreshold && forcesR(1) > m_forcesThreshold && forcesR(2) > m_forcesThreshold && forcesR(3) > m_forcesThreshold)
    return true;
  else
    return false;
}

bool WalkingModule::checkFeetVelocities()
{
  // use angular velocity
  iDynTree::Twist l_foot_twist;
  iDynTree::Twist r_foot_twist;
  
  l_foot_twist = m_FKSolver->getFrameVelocity(m_IKSolver->getLeftFootFrame());
  r_foot_twist = m_FKSolver->getFrameVelocity(m_IKSolver->getRightFootFrame());
  
//   std::cout << "====== L foot twist: " << l_foot_twist.toString() << std::endl;
//   std::cout << "====== R foot twist: " << r_foot_twist.toString() << std::endl;
  
  double linNormL = 0;
  double linNormR = 0;
  double angNormL = 0;
  double angNormR = 0;
  for(int i = 0; i < 3; i++)
  {
    linNormL += l_foot_twist.getLinearVec3()(i)*l_foot_twist.getLinearVec3()(i);
    linNormR += r_foot_twist.getLinearVec3()(i)*r_foot_twist.getLinearVec3()(i);
    angNormL += l_foot_twist.getAngularVec3()(i)*l_foot_twist.getAngularVec3()(i);
    angNormR += r_foot_twist.getAngularVec3()(i)*r_foot_twist.getAngularVec3()(i);
  }
  linNormL = std::sqrt(linNormL);
  linNormR = std::sqrt(linNormR);
  angNormL = std::sqrt(angNormL);
  angNormR = std::sqrt(angNormR);
  
  if(linNormL < m_velThreshold && linNormR < m_velThreshold && angNormL < m_velThreshold && angNormR < m_velThreshold)
    return true;
  else
    return false;
}


bool WalkingModule::checkSkinContact(std::string& link_name)
{
  return true;
}

void WalkingModule::computeInclinationPlane()
{
  iDynTree::Vector3 angles = m_inertial_R_worldFrame.asRPY();
  m_planeKx = std::tan(angles(0));
  m_planeKy = std::tan(angles(1));
//   if(m_walkingStatus == WalkingStatus::DSStable)
//     std::cout << "Inclination plane: " << m_planeKx << ", " << m_planeKy << std::endl;
}

void WalkingModule::updateOmega(double zmpX, double zmpY)
{
  double newZ = m_planeKx*zmpX + m_planeKy*zmpY;
  m_stableDCMModel->updateOmega(newZ);
  if(m_useMPC)
  {
    m_walkingController->updateOmega(newZ);
    m_walkingController->evaluateDynamics();
  } 
  else
    m_walkingDCMReactiveController->updateOmega(newZ);
}

void WalkingModule::updateOmega(iDynTree::Vector3& gravity)
{
  m_stableDCMModel->updateOmega(gravity);
  if(m_useMPC)
  {
    m_walkingController->updateOmega(gravity);
    m_walkingController->evaluateDynamics();
  } 
  else
    m_walkingDCMReactiveController->updateOmega(gravity);
}

void WalkingModule::smoothOrtTransition(iDynTree::Vector3 rpyI, iDynTree::Vector3 rpyId, std::vector <iDynTree::Rotation>& rotVec)
{
  iCub::ctrl::minJerkTrajGen ortSmoother(3,m_dT,m_IMUSmoothingTime);
  yarp::sig::Vector rpyYarp;
  yarp::sig::Vector rpyDYarp;
  yarp::sig::Vector res;
  rpyYarp.resize(3);
  rpyDYarp.resize(3);
  res.resize(3);
  rpyYarp.zero();
  rpyDYarp.zero();
  res.zero();
  for(unsigned int i = 0; i < rpyI.size(); i++)
  {
    rpyYarp(i) = rpyI(i);
    rpyDYarp(i) = rpyId(i);
  }
  ortSmoother.init(rpyYarp);
  int N = rotVec.size();
  for(unsigned int i = 0; i < N; i++)
  {
    ortSmoother.computeNextValues(rpyDYarp);
    res = ortSmoother.getPos();
    std::cout << "Ort " << i << ": " << res.toString() << std::endl;
    rotVec[i] = iDynTree::Rotation::RPY(res(0),res(1),res(2)); 
  }
  
  m_ortChangeIndex = 0;
}