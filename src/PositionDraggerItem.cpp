#include "PositionDraggerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/EigenArchive>
#include <cnoid/SimulatorItem>
#include <cnoid/EigenUtil>
#include <cnoid/SceneView>
#include <cnoid/LazyCaller>
#include <cnoid/MainWindow>

namespace cnoid {

  void PositionDraggerItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<PositionDraggerItem>("PositionDraggerItem");
  }

  PositionDraggerItem::PositionDraggerItem() {
    positionDragger_ = new PositionDragger;
    positionDragger_->setDraggerAlwaysShown(true);
    positionDragger_->sigPositionDragged().connect(std::bind(&PositionDraggerItem::onDraggerDragged, this));
    SceneView::instance()->sceneWidget()->sceneRoot()->addChild(this->positionDragger_);

    cnoid::callLater([&](){
        this->toolBar_ = new ToolBar((this->name()+"Bar").c_str());
        this->toolBar_->setVisibleByDefault(true);
        MainWindow::instance()->addToolBar(this->toolBar_);
        this->button_ = this->toolBar_->addToggleButton(QIcon(":/Base/icons/walkthrough.png"),this->linkName_.c_str());
        this->button_->sigToggled().connect([&](bool on){ onButtonToggled(on); });
      });

    SimulationBar::instance()->sigSimulationAboutToStart().connect([&](cnoid::SimulatorItem* simulatorItem){onSimulationAboutToStart(simulatorItem);});
  }

  void PositionDraggerItem::onPositionChanged(){
    this->bodyItem_ = findOwnerItem<BodyItem>();
    this->onButtonToggled(this->state_ == ENABLED);
  }

  bool PositionDraggerItem::store(Archive& archive) {
    archive.write("linkName", this->linkName_);
    write(archive,"localp", this->localT_.translation());
    write(archive,"localR", cnoid::AngleAxis(this->localT_.linear()));
    archive.write("pgain", this->pgain_);
    archive.write("dgain", this->dgain_);
    archive.write("pgainR", this->pgainR_);
    archive.write("dgainR", this->dgainR_);
    return true;
  }

  bool PositionDraggerItem::restore(const Archive& archive) {
    archive.read("linkName", this->linkName_);
    if(this->button_)this->button_->setToolTip(this->linkName_.c_str());
    cnoid::Vector3 p;
    read(archive,"localp", p);
    this->localT_.translation() = p;
    cnoid::AngleAxis angleAxis;
    read(archive,"localR", angleAxis);
    this->localT_.linear() = cnoid::Matrix3(angleAxis);
    archive.read("pgain", this->pgain_);
    archive.read("dgain", this->dgain_);
    archive.read("pgainR", this->pgainR_);
    archive.read("dgainR", this->dgainR_);
    return true;
  }

  void PositionDraggerItem::onButtonToggled(bool on){
    if(on){
      if(this->bodyItem_){
        cnoid::LinkPtr link = this->bodyItem_->body()->link(this->linkName_);
        if(link){
          this->targetT_ = link->T() * this->localT_;
          this->prevError_ = cnoid::Vector6::Zero();
          this->positionDragger_->setRadius(0.2);
          this->positionDragger_->setDraggerAlwaysShown(true);
          this->positionDragger_->T() = this->targetT_;
          state_ = ENABLED;
        }else{
          MessageView::instance()->putln(this->linkName_+" not found.", MessageView::ERROR);
        }
      }
    }else{
      this->positionDragger_->setDraggerAlwaysHidden(true);
      state_ = DISABLED;
    }
  }

  void PositionDraggerItem::onDraggerDragged() {
    SimulatorItem* activeSimulatorItem = SimulatorItem::findActiveSimulatorItemFor(bodyItem_);
    this->targetT_ = this->positionDragger_->draggedPosition();
    this->positionDragger_->T() = this->positionDragger_->draggedPosition();
  }

  void PositionDraggerItem::onSimulationAboutToStart(SimulatorItem* simulatorItem)
  {
    this->currentSimulatorItem_ = simulatorItem;
    this->currentSimulatorItemConnections_.add(
        simulatorItem->sigSimulationStarted().connect([&](){ onSimulationStarted(); }));

  }

  void PositionDraggerItem::onSimulationStarted()
  {
    this->onButtonToggled(this->state_ == ENABLED);

    this->currentSimulatorItem_->addPreDynamicsFunction([&](){ onSimulationStep(); });
  }

  void PositionDraggerItem::onSimulationStep()
  {
    double dt = this->currentSimulatorItem_->worldTimeStep();

    if(!this->bodyItem_ || this->state_==DISABLED) return;

    SimulationBodyPtr simBody = this->currentSimulatorItem_->findSimulationBody(this->bodyItem_);
    if(!simBody) return;

    cnoid::LinkPtr link = simBody->body()->link(this->linkName_);
    if(!link) return;

    cnoid::Position currentT = link->T() * this->localT_;

    cnoid::Vector6 error;
    error.head<3>() = currentT.translation() - this->targetT_.translation();
    cnoid::AngleAxis angleAxis = cnoid::AngleAxis(currentT.linear() * this->targetT_.linear().transpose());
    error.tail<3>() = angleAxis.angle()*angleAxis.axis();

    cnoid::Vector6 dError = (error - this->prevError_)/dt;
    this->prevError_ = error;

    cnoid::Vector3 f = - error.head<3>() * this->pgain_ - dError.head<3>() * this->dgain_;
    cnoid::Vector3 m = - error.tail<3>() * this->pgainR_ - dError.tail<3>() * this->dgainR_;

    link->addExternalForce(f, this->localT_.translation());
    link->tau_ext() += m;
  }

}

