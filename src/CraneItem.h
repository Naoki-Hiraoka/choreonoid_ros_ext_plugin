#ifndef CNOIDROSEXTPLUGIN_CRANE_ITEM_H
#define CNOIDROSEXTPLUGIN_CRANE_ITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cnoid/BodyItem>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <std_srvs/SetBool.h>

namespace cnoid {

  class CraneItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    CraneItem();

  protected:
    virtual void onPositionChanged() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    void setupROS(); bool setupROSDone_ = false;

    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();
    bool onLiftSrv(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    BodyItemPtr bodyItem_;
    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    ros::ServiceServer LiftSrv_;
    ros::CallbackQueue callbackQueue_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;
    double targetHeight_;
    double prevError_=0;
    enum state {UP, DOWN, DISABLED};
    state state_;
    int frame_=0;
    cnoid::Vector3 prevp_;
    cnoid::Matrix3 prevR_;

    std::string linkName_;
    cnoid::Vector3 localPos_;
    double maxHeight_;
    double minHeight_;
    double upVelocity_;
    double downVelocity_;
    double pgain_;
    double dgain_;
    double dgainR_;
    bool liftStart_ = true;
  };

  typedef ref_ptr<CraneItem> CraneItemPtr;
}

#endif
