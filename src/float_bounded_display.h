// Cheng Zhang

#ifndef FLOAT_BOUNDED_DISPLAY_H
#define FLOAT_BOUNDED_DISPLAY_H

#ifndef Q_MOC_RUN

#include "overlay_display.h"
#include <std_msgs/Float32.h>

#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/color_property.h>

#endif

namespace octopus_rviz_plugin
{
	class FloatBoundedDisplay: public OverlayDisplay {
		Q_OBJECT
	public:
		FloatBoundedDisplay();
		virtual ~FloatBoundedDisplay();

	protected:
		virtual void onInitialize();							// Inherited; Called when init
        virtual void update(float wall_dt, float ros_dt);		// Inherited; Called periodically by the visualization manager.
        virtual void onEnable();								// Derived classes override this to do the actual work of enabling themselves.
        virtual void onDisable();								// Derived classes override this to do the actual work of disabling themselves. 

        void processMessage(const std_msgs::Float32::ConstPtr& msg);
        void subscribe();
        void unsubscribe();
        ros::Subscriber sub_;

        virtual void draw();

        float data_;
        QString name_;
        float max_value_;
        float upper_bound_;
        float lower_bound_;
        float min_value_;
        QColor pointer_color_;
        QColor bound_color_;

        rviz::RosTopicProperty* topic_property_;
        rviz::StringProperty*	name_property_;
        rviz::FloatProperty* 	max_value_property_;
        rviz::FloatProperty* 	upper_bound_property_;
        rviz::FloatProperty* 	lower_bound_property_;
        rviz::FloatProperty* 	min_value_property_;
        rviz::ColorProperty*	bound_color_property_;
        rviz::ColorProperty*	pointer_color_property_;

        protected Q_SLOTS:
        virtual void updateSize();
        void updateTopic();
        void updateName();
        void updateMaxValue();
        void updateUpperBound();
        void updateLowerBound();
        void updateMinValue();
        void updatePointerColor();
        void updateBoundColor();
        virtual void updateFGAlpha();
    };

}



#endif