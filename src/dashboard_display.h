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

        rviz::RosTopicProperty* topic_property_;
        rviz::IntProperty*		width_property_;

        protected Q_SLOTS:
        void updateWidth();
        virtual void updateSize();
        void updateTopic();
        virtual void updateFGAlpha();
    };

}


#endif