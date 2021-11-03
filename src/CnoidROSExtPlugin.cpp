#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "ClockPublisherItem.h"
#include "CraneItem.h"
#include "CameraPublisherItem.h"
#include "DepthCameraPublisherItem.h"
#include "OdometryCameraPublisherItem.h"
#include "SimulatorWorldResetItem.h"
#include "PositionDraggerItem.h"

using namespace cnoid;

class ROSExtPlugin : public Plugin
{
public:

    ROSExtPlugin() : Plugin("ROSExt")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ClockPublisherItem::initializeClass(this);
      CraneItem::initializeClass(this);
      CameraPublisherItem::initializeClass(this);
      DepthCameraPublisherItem::initializeClass(this);
      OdometryCameraPublisherItem::initializeClass(this);
      SimulatorWorldResetItem::initializeClass(this);
      PositionDraggerItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSExtPlugin)
