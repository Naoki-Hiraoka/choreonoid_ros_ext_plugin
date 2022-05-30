#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <iostream>

using namespace cnoid;

class SpringDamperController : public cnoid::SimpleController
{
  class SpringDamperJoint {
  public:
    cnoid::LinkPtr joint;
    double pgain;
    double dgain;
    SpringDamperJoint(cnoid::LinkPtr joint_, double pgain_, double dgain_)
      : joint(joint_),
        pgain(pgain_),
        dgain(dgain_) {}
  };

  std::vector<SpringDamperJoint> targets;

public:

    virtual bool initialize(SimpleControllerIO* io) override
    {
      std::vector<std::string> options = io->options();
      for(size_t i=0;i+2<options.size();i+=3){
        std::string linkName = options[i];
        double pgain =  std::stod(options[i+1]);
        double dgain =  std::stod(options[i+2]);

        cnoid::LinkPtr link = io->body()->link(linkName);
        if(link){
          targets.emplace_back(link, pgain, dgain);
          link->setActuationMode(Link::JOINT_TORQUE);
          io->enableOutput(link);
          io->enableInput(link, JOINT_DISPLACEMENT | JOINT_VELOCITY);
          io->os() << "[SpringDamperController] joint: \"" << linkName <<"\" P: " << pgain << " dgain:  " << dgain << std::endl;
        }else{
          io->os() << "\e[0;31m" << "[SpringDamperController] Spring-damper joint \"" << linkName <<"\" cannot be detected." << "\e[0m" << std::endl;
        }
      }

      return true;
    }

    virtual bool control() override
    {
      for(size_t i=0;i<targets.size();i++){
        targets[i].joint->u() = - targets[i].pgain * targets[i].joint->q() - targets[i].dgain * targets[i].joint->dq();
      }
      return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SpringDamperController)
