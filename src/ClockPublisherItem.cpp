#include "ClockPublisherItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>

namespace cnoid {

  void ClockPublisherItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<ClockPublisherItem>("ClockPublisherItem");
  }

  ClockPublisherItem::ClockPublisherItem(){
    if(!ros::isInitialized()){
      QStringList argv_list = QCoreApplication::arguments();
      int argc = argv_list.size();
      char* argv[argc];
      //なぜかわからないがargv_list.at(i).toUtf8().data()のポインタをそのままargvに入れるとros::initがうまく解釈してくれない.
      for(size_t i=0;i<argv_list.size();i++){
        char* data = argv_list.at(i).toUtf8().data();
        size_t dataSize = 0;
        for(size_t j=0;;j++){
          if(data[j] == '\0'){
            dataSize = j;
            break;
          }
        }
        argv[i] = (char *)malloc(sizeof(char) * dataSize+1);
        for(size_t j=0;j<dataSize;j++){
          argv[i][j] = data[j];
        }
        argv[i][dataSize] = '\0';
      }
      ros::init(argc,argv,"choreonoid");
      for(size_t i=0;i<argc;i++){
        free(argv[i]);
      }
    }
    this->clockPub_ = ros::NodeHandle().advertise<rosgraph_msgs::Clock>("/clock", 1);
    this->publishThread_ = std::thread(std::bind(&ClockPublisherItem::publishThreadFunc, this));
    SimulationBar::instance()->sigSimulationAboutToStart().connect([&](SimulatorItem* simulatorItem){onSimulationAboutToStart(simulatorItem);});
  }

  void ClockPublisherItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;
    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect(
            [&](){ onSimulationStarted(); }));
  }

  void ClockPublisherItem::onSimulationStarted()
  {
    this->currentSimulatorItem_->addPreDynamicsFunction([&](){ onSimulationStep(); });
  }

  void ClockPublisherItem::onSimulationStep()
  {
    {
      std::lock_guard<std::mutex> lk(this->publishMtx_);

      double timestep = this->currentSimulatorItem_->worldTimeStep();
      int frame = this->currentSimulatorItem_->simulationFrame();

      unsigned long timestep_nsec = timestep * 1000000000;
      unsigned long long time_nsec = frame * timestep_nsec;

      // Publish clock
      std::shared_ptr<rosgraph_msgs::Clock> msg = std::make_shared<rosgraph_msgs::Clock>();
      msg->clock.sec = time_nsec / 1000000000;
      msg->clock.nsec = time_nsec - msg->clock.sec * 1000000000;
      this->clockMsg_ = msg;
    }
    this->publishCond_.notify_all();
  }

  void ClockPublisherItem::publishThreadFunc(){
    while(ros::ok()){
      std::shared_ptr<rosgraph_msgs::Clock> clockMsg;
      {
        std::unique_lock<std::mutex> lk(this->publishMtx_);
        this->publishCond_.wait(lk);
        clockMsg = this->clockMsg_; this->clockMsg_ = nullptr;
      }
      if(clockMsg) this->clockPub_.publish(*clockMsg);
    }
    return;
  }

}

