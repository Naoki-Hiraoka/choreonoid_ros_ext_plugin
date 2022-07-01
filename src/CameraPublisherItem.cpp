#include "CameraPublisherItem.h"
#include <QCoreApplication>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace cnoid {

  void CameraPublisherItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager().registerClass<CameraPublisherItem>("CameraPublisherItem");
  }

  CameraPublisherItem::CameraPublisherItem(){
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
  }

  void CameraPublisherItem::setupROS() {
    if(this->setupROSDone_) return;
    this->setupROSDone_ = true;

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    std::string topicName;
    if(this->imageTopicName_!="") topicName = this->imageTopicName_;
    else topicName = this->cameraName_+"/color/image_raw";
    this->pub_ = it.advertise(topicName, 1);

    std::string infoName;
    if(this->cameraInfoTopicName_!="") infoName = this->cameraInfoTopicName_;
    else infoName = this->cameraName_+"/color/camera_info";
    this->infoPub_ = nh.advertise<sensor_msgs::CameraInfo>(infoName, 1);
  }

  bool CameraPublisherItem::initialize(ControllerIO* io) {
    this->io_ = io;
    this->timeStep_ = io->worldTimeStep();

    setupROS(); // コンストラクタやcallLaterだとname()やrestore()が未完了

    return true;
  }

  bool CameraPublisherItem::start() {
    this->sensor_ = this->io_->body()->findDevice<cnoid::Camera>(this->cameraName_);
    if (this->sensor_) {
      this->sensor_->sigStateChanged().connect(boost::bind(&CameraPublisherItem::updateVisionSensor, this));
      return true;
    }else{
      this->io_->os() << "\e[0;31m" << "[CameraPublisherItem] camera [" << this->cameraName_ << "] not found"  << "\e[0m" << std::endl;
      return false;
    }
  }

  void CameraPublisherItem::updateVisionSensor() {
    std_msgs::Header header;
    header.stamp.fromSec(std::max(0.0, this->io_->currentTime() - this->sensor_->delay()));
    if(this->frameId_.size()!=0) header.frame_id = this->frameId_;
    else header.frame_id = this->sensor_->name();

    sensor_msgs::Image vision;
    {
      vision.header = header;
      vision.height = this->sensor_->image().height();
      vision.width = this->sensor_->image().width();
      if (this->sensor_->image().numComponents() == 3)
        vision.encoding = sensor_msgs::image_encodings::RGB8;
      else if (this->sensor_->image().numComponents() == 1)
        vision.encoding = sensor_msgs::image_encodings::MONO8;
      else {
        ROS_WARN("unsupported image component number: %i", this->sensor_->image().numComponents());
      }
      vision.is_bigendian = 0;
      vision.step = this->sensor_->image().width() * this->sensor_->image().numComponents();
      vision.data.resize(vision.step * vision.height);
      std::memcpy(&(vision.data[0]), &(this->sensor_->image().pixels()[0]), vision.step * vision.height);
    }
    this->pub_.publish(vision);

    sensor_msgs::CameraInfo info;
    {
      info.header = header;
      info.width  = this->sensor_->image().width();
      info.height = this->sensor_->image().height();
      info.distortion_model = "plumb_bob";
      info.K[0] = std::min(info.width, info.height) / 2 / tan(this->sensor_->fieldOfView()/2);
      info.K[2] = (this->sensor_->image().width()-1)/2.0;
      info.K[4] = info.K[0];
      info.K[5] = (this->sensor_->image().height()-1)/2.0;
      info.K[8] = 1;
      info.P[0] = info.K[0];
      info.P[2] = info.K[2];
      info.P[5] = info.K[4];
      info.P[6] = info.K[5];
      info.P[10] = 1;
      info.R[0] = info.R[4] = info.R[8] = 1;
    }
    this->infoPub_.publish(info);

  }

  bool CameraPublisherItem::store(Archive& archive) {
    archive.write("cameraName", this->cameraName_);
    archive.write("imageTopicName", this->imageTopicName_);
    archive.write("cameraInfoTopicName", this->cameraInfoTopicName_);
    archive.write("frameId", this->frameId_);
    return true;
  }

  bool CameraPublisherItem::restore(const Archive& archive) {
    archive.read("cameraName", this->cameraName_);
    archive.read("imageTopicName", this->imageTopicName_);
    archive.read("cameraInfoTopicName", this->cameraInfoTopicName_);
    archive.read("frameId", this->frameId_);
    return true;
  }

}
