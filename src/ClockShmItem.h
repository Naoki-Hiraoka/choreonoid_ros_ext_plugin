#ifndef CNOIDROSEXTPLUGIN_CLOCKSHM_ITEM_H
#define CNOIDROSEXTPLUGIN_CLOCKSHM_ITEM_H

#include <cnoid/Item>
#include <cnoid/SimulationBar>
#include <cnoid/SimulatorItem>
#include <cnoid/ConnectionSet>
#include <sys/time.h>

namespace cnoid {

  class ClockShmItem : public Item
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ClockShmItem();

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
  protected:
    void onSimulationAboutToStart(SimulatorItem* simulatorItem);
    void onSimulationStarted();
    void onSimulationStep();

    SimulatorItem* currentSimulatorItem_;
    ScopedConnectionSet currentSimulatorItemConnections_;
    struct timeval* c_shm = nullptr;
    int shmKey_ = 969;
  };

  typedef ref_ptr<ClockShmItem> ClockShmItemPtr;
}

#endif
