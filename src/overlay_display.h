// Cheng Zhang
/*
    This is the base class for other overlay display.
    Speed Display/ Speed Limit Display/ Vehicle Control Display
    have not used this parent class. If time is allowed, it is better
    to change these three classes using OverlayDisplay as parent class.

*/

#ifndef OCTOPUS_OVERLAY_DISPLAY_H
#define OCTOPUS_OVERLAY_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/display.h>

#include "overlay_utils.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>

#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/ros_topic_property.h>

#endif

namespace octopus_rviz_plugin
{
	class OverlayDisplay: public rviz::Display {
		Q_OBJECT
	public:
		OverlayDisplay();
		virtual ~OverlayDisplay();

        // methods for OverlayPickerTool
        virtual bool isInRegion(int x, int y);
        virtual void movePosition(int x, int y);
        virtual void setPosition(int x, int y);
        virtual int getX() { return left_; };
        virtual int getY() { return top_; };

    protected:
        virtual void onInitialize();							// Inherited; Called when init
        virtual void update(float wall_dt, float ros_dt);		// Inherited; Called periodically by the visualization manager.
        virtual void onEnable();								// Derived classes override this to do the actual work of enabling themselves.
        virtual void onDisable();								// Derived classes override this to do the actual work of disabling themselves. 

        virtual void initProperties();

        virtual void draw() {}

        OverlayObject::Ptr overlay_;
        bool update_required_;

        int width_;
        int height_;
        int left_;
        int top_;
        QColor fg_color_;
        QColor bg_color_;

        rviz::IntProperty*      size_property_;
        rviz::IntProperty*      left_property_;
        rviz::IntProperty*      top_property_;
        rviz::ColorProperty*    fg_color_property_;
        rviz::FloatProperty*    fg_alpha_property_;
        rviz::ColorProperty*    bg_color_property_;
        rviz::FloatProperty*    bg_alpha_property_;

        protected Q_SLOTS:
        virtual void updateSize();
        void updateLeft();
        void updateTop();
        void updateFGColor();
        virtual void updateFGAlpha();
        void updateBGColor();
        void updateBGAlpha();

    };
}

#endif