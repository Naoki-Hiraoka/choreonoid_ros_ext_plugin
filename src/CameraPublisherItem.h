#ifndef CNOIDROSEXTPLUGIN_CAMERAPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_CAMERAPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/Camera>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace cnoid {

  class CameraPublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    CameraPublisherItem();

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
    void setupROS(); bool setupROSDone_ = false;
    void updateVisionSensor();

    image_transport::Publisher pub_;
    ros::Publisher infoPub_;

    std::string cameraName_;
    std::string imageTopicName_;
    std::string cameraInfoTopicName_;
    std::string frameId_;

    cnoid::ControllerIO* io_;
    cnoid::CameraPtr sensor_;
    double timeStep_;
  };

  typedef ref_ptr<CameraPublisherItem> CameraPublisherItemPtr;
}

#endif
