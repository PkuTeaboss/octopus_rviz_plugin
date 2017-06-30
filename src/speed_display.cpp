// Cheng Zhang

#include "speed_display.h"

namespace octopus_rviz_plugin
{

  SpeedDisplay::SpeedDisplay()
  {

    Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
         // Create an overlay
    Ogre::Overlay* overlay = overlayManager.create( "OverlayName" );

         // Create a panel
    Ogre::OverlayContainer* panel = static_cast<Ogre::OverlayContainer*>( overlayManager.createOverlayElement( "Panel", "PanelName" ) );
    panel->setPosition( 0.9, 0 );
    panel->setDimensions( 0.1, 0.1 );
    panel->setMaterialName( "BaseWhite" );
         // Add the panel to the overlay
    overlay->add2D( panel );

         // Show the overlay
    overlay->show();
  }


  SpeedDisplay::~SpeedDisplay()
  {
  }
}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::SpeedDisplay,rviz::Display )
// END_TUTORIAL
