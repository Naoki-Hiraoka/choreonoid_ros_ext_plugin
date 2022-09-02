#ifndef CNOIDROSEXTPLUGIN_DEPTHCAMERAPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_DEPTHCAMERAPUBLISHER_ITEM_H

#include <cnoid/ControllerItem>
#include <cnoid/RangeCamera>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace cnoid {

  class DepthCameraPublisherItem : public ControllerItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    DepthCameraPublisherItem();

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
    void publishThreadFunc();

    std::thread publishThread_;
    std::mutex publishMtx_;
    std::condition_variable publishCond_;

    image_transport::Publisher imagePub_;
    std::shared_ptr<sensor_msgs::Image> imageMsg_;
    ros::Publisher infoPub_;
    std::shared_ptr<sensor_msgs::CameraInfo> infoMsg_;
    image_transport::Publisher depthImagePub_;
    std::shared_ptr<sensor_msgs::Image> depthImageMsg_;
    ros::Publisher depthInfoPub_;
    std::shared_ptr<sensor_msgs::CameraInfo> depthInfoMsg_;
    ros::Publisher pointCloudPub_;
    std::shared_ptr<sensor_msgs::PointCloud2> pointCloudMsg_;

    std::string cameraName_;
    std::string imageTopicName_;
    std::string cameraInfoTopicName_;
    std::string depthImageTopicName_;
    std::string depthCameraInfoTopicName_;
    std::string pointCloudTopicName_;
    std::string frameId_;
    double minDistance_ = 0.0;
    bool publishColor_ = true;
    bool publishDepth_ = true;
    bool publishPointCloud_ = true;

    cnoid::ControllerIO* io_;
    cnoid::RangeCameraPtr sensor_;
    double timeStep_;
  };

  typedef ref_ptr<DepthCameraPublisherItem> DepthCameraPublisherItemPtr;
}

#endif
