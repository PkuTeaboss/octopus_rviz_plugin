// Cheng Zhang

#include "speed_limit_display.h"
#include <rviz/uniform_string_stream.h>

#include <QPainter>
#include <QFontDatabase>

#include <iomanip>
#include <ros/package.h>

namespace octopus_rviz_plugin
{
	SpeedLimitDisplay::SpeedLimitDisplay()
	: rviz::Display(), update_required_(false)
	{
		topic_property_ = new rviz::RosTopicProperty(
			"Topic", "",
			ros::message_traits::datatype<std_msgs::String>(),
			"std_msgs::String topic to subscribe to.",
			this, SLOT( updateTopic() ));

		size_property_ = new rviz::IntProperty(
			"height", 200, "", this, SLOT(updateSize())
			);
		left_property_ = new rviz::IntProperty(
			"left", 20, "", this, SLOT(updateLeft())
			);
		top_property_ = new rviz::IntProperty(
			"top", 20, "", this, SLOT(updateTop())
			);

		alpha_property_ = new rviz::FloatProperty(
			"Opacity", 0.9, "", this,SLOT(updateAlpha())
			);

		speed_limit_ = "50";

	}

	SpeedLimitDisplay::~SpeedLimitDisplay() {
	}

	void SpeedLimitDisplay::onInitialize() {
		static int count = 0;
		rviz::UniformStringStream ss;
		ss << "SpeedLimitDisplay" << count++;
		overlay_.reset(new OverlayObject(ss.str()));

		speed_limit_image_ = QImage(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/speed_limit_sign.png");
		int id = QFontDatabase::addApplicationFont(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/HWYGWDE.TTF");
		font_family_ = QFontDatabase::applicationFontFamilies(id).at(0);

		onEnable();
		updateAlpha();
		updateSize();
		updateLeft();
		updateTop();

		overlay_->updateTextureSize(width_, height_);
		overlay_->setPosition(left_, top_);
		overlay_->setDimensions(overlay_->getTextureWidth(),
			overlay_->getTextureHeight());
		draw();
	}

	void SpeedLimitDisplay::update(float wall_dt, float ros_dt) {
		if (update_required_) {
			update_required_ = false;
			overlay_->updateTextureSize(width_, height_);
			overlay_->setPosition(left_, top_);
			overlay_->setDimensions(overlay_->getTextureWidth(),
				overlay_->getTextureHeight());
			draw();
		}
	}


	void SpeedLimitDisplay::onEnable() {
		subscribe();
		overlay_->show();
	}

	void SpeedLimitDisplay::onDisable() {
		unsubscribe();
		overlay_->hide();
	}


	void SpeedLimitDisplay::processMessage(const std_msgs::String::ConstPtr& msg) {
		if (!overlay_->isVisible()) {
			return;
		}
		if (speed_limit_ != msg->data) {
			speed_limit_ = msg->data;
			update_required_ = true;
		}
	}

	void SpeedLimitDisplay::subscribe() {
		std::string topic_name = topic_property_->getTopicStd();
		if (topic_name.length() > 0 && topic_name != "/") {
			ros::NodeHandle n;
			sub_ = n.subscribe(topic_name, 1, &SpeedLimitDisplay::processMessage, this);
		}
	}

	void SpeedLimitDisplay::unsubscribe() {
		sub_.shutdown();
	}


	void SpeedLimitDisplay::draw() {
		ScopedPixelBuffer buffer = overlay_->getBuffer();
		QColor background(0,0,0,0);
		QImage Hud = buffer.getQImage(*overlay_, background);
		QPainter painter( &Hud );
		painter.setRenderHint(QPainter::Antialiasing, true);

		// Draw Background
		painter.drawImage(QRect(0, 0, width_, height_), speed_limit_image_);

		// Draw number
		QColor color(0,0,0, alpha_ * 255.0);
		int numberSize = height_ * 0.37;
		int numberTop = height_ * 0.54;

		painter.setPen(QPen(color, 50, Qt::SolidLine));
		QFont font = painter.font();
		font.setPointSize(numberSize);
		font.setFamily(font_family_);
		//font.setBold(true);
		painter.setFont(font);

		painter.drawText(
			0, numberTop, width_, numberSize,
			Qt::AlignCenter | Qt::AlignVCenter, speed_limit_.c_str()
			);

		painter.end();


	}


	void SpeedLimitDisplay::updateTopic() {
		unsubscribe();
		subscribe();
	}


	void SpeedLimitDisplay::updateSize() {
		int size = size_property_->getInt();
		if (size <= 32 || size >= 1024) {
			return;
		}
		width_ = size * speed_limit_image_.width() / speed_limit_image_.height();
		height_ = size;
		update_required_ = true;
	}

	void SpeedLimitDisplay::updateLeft() {
		left_ = left_property_->getInt();
		update_required_ = true;
	}

	void SpeedLimitDisplay::updateTop() {
		top_ = top_property_->getInt();
		update_required_ = true;
	}

	void SpeedLimitDisplay::updateAlpha() {
		alpha_ = alpha_property_->getFloat();

		for(int i=0; i<speed_limit_image_.width(); i++){
			for(int j=0;j <speed_limit_image_.height(); j++) {
				QColor tmp = speed_limit_image_.pixel(i,j);
				tmp.setAlpha(alpha_ * 255.0);
				speed_limit_image_.setPixel(i,j, tmp.rgba());
			}
		}

		update_required_ = true;
	}

	bool SpeedLimitDisplay::isInRegion(int x, int y)
	{
		return (top_ < y && top_ + height_ > y &&
			left_ < x && left_ + width_ > x);
	}

	void SpeedLimitDisplay::movePosition(int x, int y)
	{
		top_ = y;
		left_ = x;
		update_required_ = true;
	}

	void SpeedLimitDisplay::setPosition(int x, int y)
	{
		top_property_->setValue(y);
		left_property_->setValue(x);
		update_required_ = true;
	}
}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::SpeedLimitDisplay,rviz::Display )
// END_TUTORIAL