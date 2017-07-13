// Cheng Zhang

#ifndef VEHICLE_CONTROL_DISPLAY_H
#define VEHICLE_CONTROL_DISPLAY_H

#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/BrakeReport.h>
#include <dbw_mkz_msgs/ThrottleReport.h>
#include <dbw_mkz_msgs/TurnSignalCmd.h>

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

	class VehicleControlDisplay: public rviz::Display {
		Q_OBJECT
	public:
        VehicleControlDisplay();
        virtual ~VehicleControlDisplay();

        // methods for OverlayPickerTool
        virtual bool isInRegion(int x, int y);
        virtual void movePosition(int x, int y);
        virtual void setPosition(int x, int y);
        virtual int getX() { return left_; };
        virtual int getY() { return top_; };

    protected:
       	virtual void onInitialize();						// Inherited; Called when init
       	virtual void update(float wall_dt, float ros_dt);	// Inherited; Called periodically by the visualization manager.
       	virtual void onEnable();							// Derived classes override this to do the actual work of enabling themselves.
       	virtual void onDisable();                           // Derived classes override this to do the actual work of disabling themselves. 

        void draw();
        void drawPadelBackGround(QPainter& painter, int centerX, int centerY, int startAngle);									

        std::map<std::string, ros::Subscriber> sub_map_;
        void subscribeSteering(); 
        void subscribeBrake(); 
        void subscribeThrottle(); 
        void subscribeSignal(); 
        void unsubscribe(rviz::RosTopicProperty*);
        void processSteeringMessage(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg);
        void processBrakeMessage(const dbw_mkz_msgs::BrakeReport::ConstPtr& msg);
        void processThrottleMessage(const dbw_mkz_msgs::ThrottleReport::ConstPtr& msg);
        void processSignalMessage(const dbw_mkz_msgs::TurnSignalCmd::ConstPtr& msg);


        OverlayObject::Ptr overlay_;
        bool update_required_;
        QImage wheel_image_;
        QImage signal_off_image_;
        QImage signal_on_image_;

        int wheel_angle_;
        int throttle_angle_;
        int brake_angle_;
        uint8_t signal_;


        int width_;
        int height_;
        int left_;
        int top_;
        QColor pointer_color_;
        QColor fg_color_;
        QColor throttle_color_;
        QColor brake_color_;
        QColor signal_on_color_;
        QColor signal_off_color_;
        QColor bg_color_;

        rviz::RosTopicProperty* steering_topic_property_;
        rviz::RosTopicProperty* brake_topic_property_;
        rviz::RosTopicProperty* throttle_topic_property_;
        rviz::RosTopicProperty* signal_topic_property_;
        rviz::IntProperty*		size_property_;
        rviz::IntProperty*		left_property_;
        rviz::IntProperty*		top_property_;
        rviz::ColorProperty*	pointer_color_property_;
        rviz::ColorProperty* 	fg_color_property_;
        rviz::ColorProperty*    throttle_color_property_;
        rviz::ColorProperty*    brake_color_property_;
        rviz::ColorProperty*    signal_color_property_;
        rviz::FloatProperty* 	fg_alpha_property_;
        rviz::ColorProperty* 	bg_color_property_;
        rviz::FloatProperty* 	bg_alpha_property_;


        protected Q_SLOTS:
        void updateSteeringTopic();
        void updateBrakeTopic();
        void updateThrottleTopic();
        void updateSignalTopic();
        void updateSize();
        void updateLeft();
        void updateTop();
        void updatePointerColor();
        void updateFGColor();
        void updateThrottleColor();
        void updateBrakeColor();
        void updateSignalColor();
        void updateFGAlpha();
        void updateBGColor();
        void updateBGAlpha();

    };

}

#endif 
