#ifndef CNOIDROSEXTPLUGIN_CLOCKPUBLISHER_ITEM_H
#define CNOIDROSEXTPLUGIN_CLOCKPUBLISHER_ITEM_H

#include <cnoid/Item>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace cnoid {

  class ClockPublisherItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ClockPublisherItem();

  protected:
    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();
    void publishThreadFunc();

    std::thread publishThread_;
    std::mutex publishMtx_;
    std::condition_variable publishCond_;

    ros::Publisher clockPub_;
    std::shared_ptr<rosgraph_msgs::Clock> clockMsg_;
    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
  };

  typedef ref_ptr<ClockPublisherItem> ClockPublisherItemPtr;
}

#endif
