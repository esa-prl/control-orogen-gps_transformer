#pragma once

#include "gps_transformer/TaskBase.hpp"
#include <random>

namespace gps_transformer {

using Pose = Eigen::Affine3d;
using BasePose = base::samples::RigidBodyState;

class Task : public TaskBase {
  public:
    explicit Task(std::string const& name = "gps_transformer::Task")
            : TaskBase(name),
              lastPose_(Pose::Identity()) {}

  protected:
    void inputPoseTransformerCallback(
            const base::Time& timestamp,
            const BasePose& gpsBasePose) override {
        if (!initialBasePose_.hasValidPosition()) {
            initialBasePose_ = gpsBasePose;
            lastDriftPose_ = Pose::Identity();
            first = true;
            traversedDistance_ = 0.0;
            absoluteDeltaYaw_ = 0.0;
            accumulatedDeltaYaw_ = 0.0;
        }

        BasePose diffPose = gpsBasePose;
        diffPose.position -= initialBasePose_.position;

        Pose yawRot, pose, deltaPose, driftPose;
        yawRot = Eigen::AngleAxisd(_yawOffset.get(), Eigen::Vector3d::UnitZ());
        pose = yawRot * diffPose.getTransform();
        deltaPose = lastPose_.inverse() * pose;
        lastPose_ = pose;
        driftPose = lastDriftPose_ * deltaPose;

        if (first) {
            first = false;
        }
        else { //Add random noise to the pose that represents a drift of 1.5% of the traversed distance
            std::uniform_real_distribution<> distribution(0.005, 0.025);
            std::uniform_real_distribution<> distribution2(-1, 1);
            double drift = distribution(generator_);
            double sign = distribution2(generator_);
            sign = sign > 0 ? 1.0 : -1.0;
            drift = sign*drift;
//            std::cout << "Drift: " << drift << std::endl;
            driftPose.translation() += driftPose.linear() * (drift*deltaPose.translation());
//            deltaPose.translation() += drift*deltaPose.translation();
            double deltaYaw;
            BasePose baseDeltaPose;
            baseDeltaPose.setTransform(deltaPose);
            deltaYaw=baseDeltaPose.getYaw();
//            std::cout << "DeltaYaw: " << deltaYaw << std::endl;
            accumulatedDeltaYaw_ += fabs(deltaYaw)*(180.0/3.141592);
            absoluteDeltaYaw_ += deltaYaw*(180.0/3.141592);
//            std::cout << "TotalDeltaYaw: " << totalDeltaYaw_ << std::endl;
            _accumulatedDeltaYaw.write(accumulatedDeltaYaw_);
            _absoluteDeltaYaw.write(absoluteDeltaYaw_);
            yawRot = Eigen::AngleAxisd((15*drift*deltaYaw), Eigen::Vector3d::UnitZ());
//            std::cout << "DriftDeltaYaw: " << drift*deltaYaw << std::endl;
            driftPose = driftPose * yawRot;
//            deltaPose = yawRot * deltaPose;
        }
        lastDriftPose_ = driftPose;

        BasePose outputBaseDeltaPose, outputBasePose, outputDriftBasePose;
        outputBaseDeltaPose.setTransform(deltaPose);
        outputBasePose.setTransform(pose);
        outputDriftBasePose.setTransform(driftPose);

        _outputDeltaPose.write(outputBaseDeltaPose);
        _outputPose.write(outputBasePose);
        _outputDriftPose.write(outputDriftBasePose);

        const Eigen::Vector3d deltaXYZ = deltaPose.translation();
        const Eigen::Vector2d deltaXY = deltaXYZ.head(2);
        traversedDistance_ += deltaXY.norm();
        _traversedDistance.write(traversedDistance_);
        double estimationError = sqrt(pow(outputDriftBasePose.position.x()-outputBasePose.position.x(),2) + pow(outputDriftBasePose.position.y()-outputBasePose.position.y(),2));
        _odometryError.write(estimationError);

   }

  protected:
    BasePose initialBasePose_;
    Pose lastPose_, lastDriftPose_;
    std::default_random_engine generator_;
    bool first;
    double traversedDistance_;
    double absoluteDeltaYaw_;
    double accumulatedDeltaYaw_;
};

}  // namespace gps_transformer

