// Cheng Zhang

#ifndef SPEED_DISPLAY_H
#define SPEED_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/display.h>

#include <OGRE/Overlay/OgrePanelOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayElement.h>
#include <OGRE/Overlay/OgreOverlayContainer.h>
#include <OGRE/Overlay/OgreOverlayManager.h>

#endif


namespace octopus_rviz_plugin
{

  class SpeedDisplay: public rviz::Display
  {
    Q_OBJECT
  public:
    SpeedDisplay();
    virtual ~SpeedDisplay();

  private:

  };

}

#endif 
