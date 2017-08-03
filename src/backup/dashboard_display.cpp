// Cheng Zhang

#include "dashboard_display.h"

#include <QPainter>

namespace octopus_rviz_plugin
{
	DashboardDisplay::DashboardDisplay()
	: OverlayDisplay()
	{
		topic_property_ = new rviz::RosTopicProperty(
			"Topic", "",
			ros::message_traits::datatype<dashboard>(),
			"octopus_rviz_plugin::dashboard topic to subscribe to.",
			this, SLOT( updateTopic() ));

		OverlayDisplay::initProperties();

		size_property_->setValue(300);
		fg_color_property_->setValue(QColor(69, 193, 251));
	}

	DashboardDisplay::~DashboardDisplay(){
		data_.reset();
	}

	void DashboardDisplay::onInitialize() {
		OverlayDisplay::onInitialize();
	}

	void DashboardDisplay::update(float wall_dt, float ros_dt) {
		OverlayDisplay::update(wall_dt, ros_dt);
	}

	void DashboardDisplay::onEnable() {
		subscribe();
		OverlayDisplay::onEnable();
	}

	void DashboardDisplay::onDisable() {
		unsubscribe();
		OverlayDisplay::onDisable();
	}

	void DashboardDisplay::processMessage(const dashboard::ConstPtr& msg) {
		if (!overlay_->isVisible()) {
			return;
		}
		data_.reset(new dashboard(*(msg)));
		update_required_ = true;
	}

	void DashboardDisplay::subscribe() {
		std::string topic_name = topic_property_->getTopicStd();
		if (topic_name.length() > 0 && topic_name != "/") {
			ros::NodeHandle n;
			sub_ = n.subscribe(topic_name, 1, &DashboardDisplay::processMessage, this);
		}
	}

	void DashboardDisplay::unsubscribe() {
		sub_.shutdown();
	}

	void drawRadiusLine(QPainter& painter, int centerX, int centerY, int innerR, int outerR, int degree) {
		int innerX = centerX + innerR * (cos(degree * 3.14 / 180));
		int innerY = centerY + innerR * (-sin(degree * 3.14 / 180));
		int outerX = centerX + outerR * (cos(degree * 3.14 / 180));
		int outerY = centerY + outerR * (-sin(degree * 3.14 / 180));
		painter.drawLine(innerX, innerY, outerX, outerY);
	}


	void drawSpeed(QPainter& painter,float speed, int centerX, int centerY, int width, QColor& fg_color, QColor& pointer_color) {
		int max_speed = 140;
		int interval = 5;
		int textCount = 4;
		float min_degree = 60;
		float max_degree = 300;
		int scaleInnerR1 = width / 2 - 9 * width / 128;
		int scaleInnerR2 = width / 2 - 6 * width / 128;
		int scaleOuterR = width / 2 - 2 * width / 128;
		int scaleWidth = 3 * width / 200;
		int textR = width / 2 - 19 * width / 128;
		int textSize = 8 * width / 128;
		int speedSize = 14 * width / 128;
		int pointerInnerR = width / 2 - 35 * width / 128;
		int pointerOuterR = width / 2 - 10;
		int pointerWidth = 2 * width / 200;

		int n = max_speed / interval;
		float interval_degree = (max_degree - min_degree) / n;

		// Draw BackGround Dash Board
		painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine));
		QFont font = painter.font();
		font.setPointSize(textSize);
		font.setBold(true);
		painter.setFont(font);

		for (int i=0; i <= n; ++i) {
			float degree = min_degree + interval_degree * i - 90;
			if (i % textCount == 0) {
				int textX = centerX + textR * (-cos(degree * 3.14 / 180));
				int textY = centerY + textR * (-sin(degree * 3.14 / 180));
				painter.drawText(
					textX - textSize * 1.3, textY - textSize/2, textSize*2.5, textSize,
					Qt::AlignCenter | Qt::AlignVCenter, QString::number(interval * i));

				drawRadiusLine(painter, centerX, centerY, scaleInnerR1, scaleOuterR, degree);
			} else {
				drawRadiusLine(painter, centerX, centerY, scaleInnerR2, scaleOuterR, degree);
			}
		}

		// Draw Speed
		painter.drawText(
			centerX - textSize * 3, centerY, textSize*6, textSize,
			Qt::AlignCenter | Qt::AlignVCenter, "MPH"
			);

		font.setPointSize(speedSize);
		painter.setFont(font);
		painter.drawText(
			centerX - speedSize * 3, centerY - width * 0.12 - speedSize/2, speedSize*6, speedSize,
			Qt::AlignCenter | Qt::AlignVCenter, QString::number(speed, 'f', 1)
			);
		

		// Draw Pointer
		if (speed < 0) speed = 0;
		else if (speed > max_speed) speed = max_speed;

		painter.setPen(QPen(pointer_color, pointerWidth, Qt::SolidLine));
		int degree = min_degree + round(speed / max_speed * (max_degree - min_degree));
		drawRadiusLine(painter, centerX, centerY, pointerInnerR, pointerOuterR, degree);
	}


	float getDegree(float value, float lower_bound, float upper_bound, int beginAngle, int endAngle) {
		return (value - lower_bound) / (upper_bound - lower_bound) * (endAngle - beginAngle) + beginAngle;
	}


	// bounds in increasing order
	void drawBoundedBar(QPainter& painter, float value, float bounds[4],
		int centerX, int centerY, int width, int beginAngle, int endAngle,
		QColor& fg_color, QColor& bound_color, QColor& fill_color) {

		if (!(bounds[0] <= bounds[1] && bounds[1] < bounds[2] && bounds[2] <= bounds[3] ) ) {
			ROS_ERROR("[octopus_rviz_plugin::dashboard] It should be lower_bound < lower_comfort < upper_comfort < upper_bound");
			ROS_ERROR("[%f, %f, %f, %f]", bounds[0], bounds[1], bounds[2], bounds[3]);
			return;
		}

		// Draw Background Dashboard
		int scaleInnerR = width / 2 - 10 * width / 128;
		int scaleOuterR = width / 2 - 2 * width / 128;
		int scaleWidth = width * 0.007;
		int boundWidth = width * 0.015;

		for (int i=0; i<4; ++i) {
			if (i == 1 || i == 2) {
				painter.setPen(QPen(bound_color, boundWidth, Qt::SolidLine, Qt::FlatCap));
			} else {
				painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
			}

			drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(bounds[i], bounds[0], bounds[3], beginAngle, endAngle));
		}
		painter.setPen(QPen(fill_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
		drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(0, bounds[0], bounds[3], beginAngle, endAngle));

		painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
		int angle_starts = beginAngle < endAngle ? beginAngle : endAngle;
		int angle_length = abs(beginAngle - endAngle);
		painter.drawArc(centerX-scaleInnerR, centerY-scaleInnerR, scaleInnerR*2, scaleInnerR*2, angle_starts*16-1, angle_length*16);
		painter.drawArc(centerX-scaleOuterR, centerY-scaleOuterR, scaleOuterR*2, scaleOuterR*2, angle_starts*16-1, angle_length*16);
	}

	void DashboardDisplay::draw() {
		ScopedPixelBuffer buffer = overlay_->getBuffer();
		QImage Hud = buffer.getQImage(*overlay_, bg_color_);
		QPainter painter( &Hud );
		painter.setRenderHint(QPainter::Antialiasing, true);

		QColor pointer_color(220, 203, 214, fg_color_.alpha());
		drawSpeed(painter, data_->speed, height_/2, height_/2, height_ * 0.75, fg_color_, pointer_color);

		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_->y_accel_lower_bound, data_->y_accel_lower_comfort, data_->y_accel_upper_comfort, data_->y_accel_upper_bound};
			drawBoundedBar(painter, data_->y_accel_data, bounds,
				height_/2, height_/2, height_, 210, 120, fg_color_, bound_color, fill_color);
		}

		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_->y_jerk_lower_bound, data_->y_jerk_lower_comfort, data_->y_jerk_upper_comfort, data_->y_jerk_upper_bound};
			drawBoundedBar(painter, data_->y_jerk_data, bounds,
				height_/2, height_/2, height_, -30, 60, fg_color_, bound_color, fill_color);
		}

		painter.end();
	}



	void DashboardDisplay::updateSize() {
		int size = size_property_->getInt();
		if (size <= 32 || size >= 1024) {
			return;
		}
		width_ = 3 * size_property_->getInt();
		height_ = size_property_->getInt();
		update_required_ = true;
	}

	void DashboardDisplay::updateTopic() {
		unsubscribe();
		subscribe();
	}

}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::DashboardDisplay,rviz::Display )
// END_TUTORIAL