// Cheng Zhang

#ifndef SPEED_LIMIT_DISPLAY_H
#define SPEED_LIMIT_DISPLAY_H

#include <std_msgs/String.h>

#ifndef Q_MOC_RUN

#include <rviz/display.h>

#include "overlay_utils.h"
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>

#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>

#endif

namespace octopus_rviz_plugin
{
	class SpeedLimitDisplay: public rviz::Display {
		Q_OBJECT
	public:
		SpeedLimitDisplay();
		virtual ~SpeedLimitDisplay();

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

		void processMessage(const std_msgs::String::ConstPtr& msg);
		void subscribe();
		void unsubscribe();


		void draw();

		ros::Subscriber sub_;
		OverlayObject::Ptr overlay_;
		bool update_required_;
		QImage speed_limit_image_;
		QString font_family_;

		std::string speed_limit_;

		bool stick_top_right_;
		int width_;
		int height_;
		int left_;
		int top_;

		float alpha_;

		int saved_left_;
		int saved_top_;

		rviz::RosTopicProperty* topic_property_;
		rviz::BoolProperty*		stick_top_right_property_;
		rviz::IntProperty*		size_property_;
		rviz::IntProperty*		left_property_;
		rviz::IntProperty*		top_property_;
		rviz::FloatProperty* 	alpha_property_;

		protected Q_SLOTS:
		void updateTopic();
		void updateStickTopRight();
		void updateSize();
		void updateLeft();
		void updateTop();
		void updateAlpha();
	};
}


#endif