#ifndef CNOIDROSEXTPLUGIN_ODOMETRYPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_ODOMETRYPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/Camera>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace cnoid {

  class OdometryPublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    OdometryPublisherItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override {}
    virtual bool control() override;
    virtual void output() override {}
    virtual void stop() override {}

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

  protected:
    void setupROS(); bool setupROSDone_ = false;
    void publishThreadFunc();

    std::thread publishThread_;
    std::mutex publishMtx_;
    std::condition_variable publishCond_;

    ros::Publisher odometryPub_;
    std::shared_ptr<nav_msgs::Odometry> odometryMsg_;

    std::string targetName_;
    std::string odometryTopicName_;
    std::string frameId_;
    std::string childFrameId_;
    double poseCovariance_ = 0.0;
    double twistCovariance_ = 0.0;
    double publishRate_ = 100.0;

    cnoid::ControllerIO* io_;
    cnoid::CameraPtr sensor_;
    cnoid::LinkPtr link_;
    double timeStep_;
    cnoid::Position prevPose_ = cnoid::Position::Identity();

    double time_ = 0.0;
  };

  typedef ref_ptr<OdometryPublisherItem> OdometryPublisherItemPtr;
}

#endif
