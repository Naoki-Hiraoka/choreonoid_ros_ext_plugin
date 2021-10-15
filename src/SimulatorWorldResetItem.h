#ifndef CNOIDROSEXTPLUGIN_SIMULATORWORLDRESET_ITEM_H
#define CNOIDROSEXTPLUGIN_SIMULATORWORLDRESET_ITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <cnoid/BodyItem>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <cnoid/Archive>
#include <std_srvs/Trigger.h>

namespace cnoid {

  class SimulatorWorldResetItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    SimulatorWorldResetItem();

  protected:
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();

    bool onResetSrv(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    ros::ServiceServer ResetSrv_;
    ros::CallbackQueue callbackQueue_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    int resetStep_=0;
  };

  typedef ref_ptr<SimulatorWorldResetItem> SimulatorWorldResetItemPtr;
}

#endif
