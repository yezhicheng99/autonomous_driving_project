/*
 *    Filename: AckermannDrivePlugin.cpp
 *  Created on: Jun 2, 2020
 *      Author: nix <daria@cogniteam.com>
 */

#include <AckermannDrivePlugin.h>
#include <string>
#include <iostream>


namespace gazebo{


AckermannDrivePlugin::AckermannDrivePlugin() : 
    
    steeringfrontLeftPid_(10, 0.0, 0.01),
    steeringfrontRightPid_(10, 0.0, 0.01),
    previousVelocity_(0),
    linearZeroCounter_(0)
    {
    steeringfrontLeftPid_.SetCmdMax(10);
    steeringfrontRightPid_.SetCmdMax(10);

    steeringfrontLeftPid_.SetCmdMin(-10);
    steeringfrontRightPid_.SetCmdMin(-10);
    
    currentCommand_.drive.speed = 0.0;
    currentCommand_.drive.steering_angle = 0.0;

}

AckermannDrivePlugin::~AckermannDrivePlugin() {

}

void AckermannDrivePlugin::commandCallback(
    const ackermann_msgs::AckermannDriveStamped::Ptr& cmd) {

    currentCommand_ = *cmd;

    double speed = fabs(currentCommand_.drive.speed);

    //
    // Speed constraints
    //

    if (speed < minSpeed_ && speed > 0) {
        currentCommand_.drive.speed = minSpeed_;
    }

    currentCommand_.drive.speed = copysign(min(speed, maxSpeed_), cmd->drive.speed);
}

void AckermannDrivePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    initParams(sdf);

    
    ros::NodeHandle node(robotNamespace_);

    cmdSubscriber_ = node.subscribe(
        driveTopic_, 2, &AckermannDrivePlugin::commandCallback, this);

    odomPublisher_ = node.advertise<nav_msgs::Odometry>(odomTopic_, 5, false);
  
    this->model_ = model;

    if(!model_) {
        return;
    }

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontRightPid_);

    this->model_->GetJointController()->SetPositionPID(
        this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                steeringfrontLeftPid_);
        
    this->model_->GetJointController()->SetPositionTarget(
        frontRightWheelSteeringJoint_, 0.0);

    this->model_->GetJointController()->SetPositionTarget(
        frontLeftWheelSteeringJoint_, 0.0);

    for (auto&& wheelJoint : wheelJoints_) {
        this->model_->GetJoint(wheelJoint)->SetParam(
            "fmax", 0, (double)torque_);
        this->model_->GetJoint(wheelJoint)->SetParam(
            "vel", 0, 0.0);

    }

    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&AckermannDrivePlugin::Update, this, std::placeholders::_1));

    ROS_INFO("AckermannDrive plugin loaded!");
    
}

void AckermannDrivePlugin::Update(const common::UpdateInfo &info) {

    
     if (lastUpdate_ == common::Time()) {
        lastUpdate_ = info.simTime;
        return;
    }

    double timeDeltaSec = (info.simTime - lastUpdate_).Double();
    if (timeDeltaSec > 0.02) {

        //
        // Steering constraints
        //

        auto steering = fmax(-0.5, fmin(0.5, currentCommand_.drive.steering_angle));

        auto linearVelocity = (double)currentCommand_.drive.speed / wheelRadius_;

        bool isRotationOnly = (double)fabs(currentCommand_.drive.speed) < (minSpeed_ / 2) &&
            fabs(steering) > 0.02;

        bool stop = fabs(currentCommand_.drive.speed) < (minSpeed_ / 2) && 
            fabs(steering) < 0.02;

        if (isRotationOnly) {
            linearZeroCounter_ += 0.04;
            linearVelocity = maxSpeed_/wheelRadius_ * 0.1 * sin(linearZeroCounter_);
            linearVelocity *= 0.99;

            if (linearVelocity < 0) {
                steering *= -1;
            }
        }

        for (auto&& wheelJoint : wheelJoints_) {

            if(!model_) {
                return;
            }

            if (stop) {
            this->model_->GetJoint(wheelJoint)->SetParam(
                "fmax", 0, (double)torque_ * 100);
            } else {
                this->model_->GetJoint(wheelJoint)->SetParam(
                    "fmax", 0, (double)torque_);
            }

            this->model_->GetJoint(wheelJoint)->SetParam("vel", 0, linearVelocity);
        }

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(frontRightWheelSteeringJoint_)->GetScopedName(), 
                    rightWheelSteering((double)steering));

        this->model_->GetJointController()->SetPositionTarget(
            this->model_->GetJoint(frontLeftWheelSteeringJoint_)->GetScopedName(), 
                    leftWheelSteering((double)steering));

        publishOdometry();

        ros::spinOnce();

        lastUpdate_ = info.simTime;
    }
}

inline double AckermannDrivePlugin::rightWheelSteering(double baseAngle) {
    return atan(2 * wheelSeparartion_ * sin(baseAngle)/ (
            2 * wheelSeparartion_ * cos(baseAngle) - 
                wheelBase_ * sin(baseAngle)));
}

inline double AckermannDrivePlugin::leftWheelSteering(double baseAngle) {
    return atan(2 * wheelSeparartion_ * sin(baseAngle)/ (
            2 * wheelSeparartion_ * cos(baseAngle) + 
                wheelBase_ * sin(baseAngle)));
}

void AckermannDrivePlugin::publishOdometry() {

    if(!model_) {
        return;
    }

    if(this->model_->GetLink(baseLink_) == NULL) {
        return;
    }
    auto position = this->model_->GetLink(baseLink_)->WorldPose().Pos();
    auto rotation = this->model_->GetLink(baseLink_)->WorldPose().Rot();

    //
    // Odometry noise
    //

    if (currentCommand_.drive.speed !=0) {
        xPoseAccumulateError_ += odomNoise_.gaussian(
            mean_, stddev_) * fabs(this->model_->GetLink(baseLink_)->WorldLinearVel().X());

        yPoseAccumulateError_ += odomNoise_.gaussian(
            mean_, stddev_) * fabs(this->model_->GetLink(baseLink_)->WorldLinearVel().Y());
    }

    auto xPose = position.X() + xPoseAccumulateError_;
    auto yPose = position.Y() + yPoseAccumulateError_;

    yawAccumulateError_ += odomNoise_.gaussian(
        mean_, stddev_) * fabs(this->model_->GetLink(baseLink_)->WorldAngularVel().Z());

    //
    //Tf publishing
    //

    double roll, pitch, yaw;
    tf::Quaternion q(rotation.X(), rotation.Y(), rotation.Z(), rotation.W());
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    q.setRPY(roll, pitch, yaw + yawAccumulateError_);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(xPose, yPose, position.Z()));

    transform.setRotation(q);

    tfBroadcaster_.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), odomFrame_, baseFrame_));

    //
    //Odom ros message publishing
    //

    nav_msgs::Odometry odomMsg;

    odomMsg.header.frame_id = odomFrame_;
    odomMsg.child_frame_id = baseFrame_;
    odomMsg.header.stamp = ros::Time::now();

    odomMsg.pose.pose.position.x = xPose;
    odomMsg.pose.pose.position.y = yPose;
    odomMsg.pose.pose.position.z = position.Z();

    tf::quaternionTFToMsg(q, odomMsg.pose.pose.orientation);

    odomMsg.twist.twist.linear.x = this->model_->GetLink(baseLink_)->WorldLinearVel().X();
    odomMsg.twist.twist.linear.y = this->model_->GetLink(baseLink_)->WorldLinearVel().Y();
    odomMsg.twist.twist.linear.z = this->model_->GetLink(baseLink_)->WorldLinearVel().Z();

    odomMsg.twist.twist.angular.x = this->model_->GetLink(baseLink_)->WorldAngularVel().X();
    odomMsg.twist.twist.angular.y = this->model_->GetLink(baseLink_)->WorldAngularVel().Y();
    odomMsg.twist.twist.angular.z = this->model_->GetLink(baseLink_)->WorldAngularVel().Z();

    odomPublisher_.publish(odomMsg);

}

void AckermannDrivePlugin::initParams(sdf::ElementPtr sdf) {

    robotNamespace_ = getParam<std::string>("robotNamespace", "", sdf);
    wheelRadius_ = getParam<double>("wheelRadius", 0.029, sdf);

    wheelJoints_.push_back(getParam<std::string>(
        "frontLeftWheelJoint", "front_left_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "frontRightWheelJoint", "front_right_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "reartLeftWheelJoint", "rear_left_wheel_joint", sdf));
    wheelJoints_.push_back(getParam<std::string>(
        "rearRightWheelJoint", "rear_right_wheel_joint", sdf));

    frontRightWheelSteeringJoint_ = getParam<std::string>(
        "frontRightWheelSteeringJoint", "front_right_wheel_steering_joint", sdf);
    frontLeftWheelSteeringJoint_ = getParam<std::string>(
        "frontLeftWheelSteeringJoint", "front_left_wheel_steering_joint", sdf);

    torque_ = getParam<double>("torque", 0.01, sdf);
    wheelSeparartion_ = getParam<double>("wheelSeparation", 0.17, sdf);
    wheelBase_ = getParam<double>("wheelBase", 0.166, sdf);
    mean_ = getParam<double>("odomNoiseMean", 0.0, sdf);
    stddev_ = getParam<double>("odomNoiseStddev", 0.01, sdf);
    minSpeed_ = getParam<double> ("minSpeed", -1.2, sdf);
    maxSpeed_ = getParam<double> ("maxSpeed", 1.2, sdf);

    baseFrame_ = getParam<std::string>("baseFrame", "base_link", sdf);
    odomFrame_ = getParam<std::string>("odomFrame", "odom", sdf);

    odomTopic_ = getParam<std::string>("odomTopic", "odom", sdf);
    driveTopic_ = getParam<std::string>("driveTopic", "ackermann_cmd", sdf);

    baseLink_ = getParam<std::string>("robotBaseLink", "base_link", sdf);
    hamsterRotationPattern_ = getParam<bool>("hamsterRotationPattern", false, sdf);

    if (!robotNamespace_.empty()) {

        for (auto&& wheelJoint : wheelJoints_) {
            wheelJoint = robotNamespace_ + "::" + wheelJoint;
        }

        frontRightWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontRightWheelSteeringJoint_; 
        frontLeftWheelSteeringJoint_ =
                robotNamespace_ + "::" + frontLeftWheelSteeringJoint_;

        baseFrame_ = tf::resolve(robotNamespace_, baseFrame_);
        odomFrame_ = tf::resolve(robotNamespace_, odomFrame_);

        baseLink_ = robotNamespace_ + "::" + baseLink_;
    }
}

} // namespace gazebo


