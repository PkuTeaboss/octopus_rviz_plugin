// Cheng Zhang

#ifndef DASHBOARD_DISPLAY_H
#define DASHBOARD_DISPLAY_H

#ifndef Q_MOC_RUN

#include "overlay_display.h"
#include <octopus_rviz_plugin/dashboard.h>

#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/color_property.h>

#endif

namespace octopus_rviz_plugin
{
	class DashboardDisplay: public OverlayDisplay {
		Q_OBJECT
	public:
		DashboardDisplay();
		virtual ~DashboardDisplay();

	protected:
		virtual void onInitialize();							// Inherited; Called when init
        virtual void update(float wall_dt, float ros_dt);		// Inherited; Called periodically by the visualization manager.
        virtual void onEnable();								// Derived classes override this to do the actual work of enabling themselves.
        virtual void onDisable();								// Derived classes override this to do the actual work of disabling themselves. 

        void processMessage(const dashboard::ConstPtr& msg);
        void subscribe();
        void unsubscribe();
        ros::Subscriber sub_;

        virtual void draw();

        dashboard data_;
        QImage wheel_image_;
        QImage signal_on_image_;
        QImage signal_off_image_;
        bool stick_bottom_;

        int saved_width_;
        int saved_left_;
        int saved_top_;
        int horizontal_padding_;

        rviz::RosTopicProperty* topic_property_;
        rviz::IntProperty*		width_property_;
        rviz::BoolProperty*     stick_bottom_property_;
        rviz::BoolProperty*		full_width_property_;

        protected Q_SLOTS:
        void updateStickBottom();
        void updateWidth();
        virtual void updateSize();
        void updateTopic();
        virtual void updateFGAlpha();
    };

}


#endif