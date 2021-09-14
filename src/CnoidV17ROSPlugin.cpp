#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "ClockPublisherItem.h"

using namespace cnoid;

class V17ROSPlugin : public Plugin
{
public:

    V17ROSPlugin() : Plugin("V17ROS")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ClockPublisherItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(V17ROSPlugin)
