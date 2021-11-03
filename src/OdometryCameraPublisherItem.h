#ifndef CNOIDROSEXTPLUGIN_ODOMETRYCAMERAPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_ODOMETRYCAMERAPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/Camera>
#include <ros/ros.h>

namespace cnoid {

  class OdometryCameraPublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    OdometryCameraPublisherItem();
    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;

    virtual double timeStep() const override { return timeStep_;};
    virtual void input() override {}
    virtual bool control() override { return true;}
    virtual void output() override {}
    virtual void stop() override {}

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

  protected:
    void updateVisionSensor();

    ros::Publisher pub_;

    std::string cameraName_;
    std::string odometryTopicName_;
    std::string frameId_;
    std::string childFrameId_;
    double poseCovariance_;
    double twistCovariance_;

    cnoid::ControllerIO* io_;
    cnoid::CameraPtr sensor_;
    double timeStep_;
    cnoid::Position prevPose_ = cnoid::Position::Identity();
  };

  typedef ref_ptr<OdometryCameraPublisherItem> OdometryCameraPublisherItemPtr;
}

#endif
