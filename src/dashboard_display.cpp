// Cheng Zhang

#include "dashboard_display.h"

#include <QPainter>

#include <ros/package.h>

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
		bg_color_property_->setValue(QColor(0, 0, 90));
	}

	DashboardDisplay::~DashboardDisplay(){

	}

	void DashboardDisplay::onInitialize() {
		wheel_image_  = QImage(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/wheel.png");
		signal_off_image_ = QImage(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/arrow.png");
		signal_on_image_ = signal_off_image_;

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
		data_ = dashboard(*(msg));
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


	void drawSpeed(QPainter& painter,float speed, int centerX, int centerY, int width, QColor& fg_color, QColor& pointer_color)
	{
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
		int degree = 270 - ( min_degree + round(speed / max_speed * (max_degree - min_degree)));
		drawRadiusLine(painter, centerX, centerY, pointerInnerR, pointerOuterR, degree);
	}


	float getDegree(float value, float lower_bound, float upper_bound, int beginAngle, int endAngle) {
		return (value - lower_bound) / (upper_bound - lower_bound) * (endAngle - beginAngle) + beginAngle;
	}


	// bounds in increasing order
	void drawBoundedBar(QPainter& painter, float value, float bounds[4],
		int centerX, int centerY, int width, int beginAngle, int endAngle,
		QColor& fg_color, QColor& bound_color, QColor& fill_color) 
	{

		if (!(bounds[0] <= bounds[1] && bounds[1] < bounds[2] && bounds[2] <= bounds[3]) ) {
			if (!(bounds[0]==0 && bounds[1]==0 && bounds[2]==0 && bounds[3]==0)) {
				ROS_ERROR("[octopus_rviz_plugin::dashboard] It should be lower_bound < lower_comfort < upper_comfort < upper_bound");
				ROS_ERROR("[%f, %f, %f, %f]", bounds[0], bounds[1], bounds[2], bounds[3]);
			}
			return;
		}

		int scaleInnerR = width / 2 - 10 * width / 128;
		int scaleOuterR = width / 2 - 2 * width / 128;
		int fillR = (scaleInnerR + scaleOuterR) / 2; 
		int scaleWidth = width * 0.007;
		int boundWidth = width * 0.015;
		int fillWidth = 8 * width / 128;

		// Draw value
		painter.setPen(QPen(fill_color, fillWidth, Qt::SolidLine, Qt::FlatCap));
		int angle_zero = getDegree(0, bounds[0], bounds[3], beginAngle, endAngle);
		int angle_value = getDegree(value, bounds[0], bounds[3], beginAngle, endAngle);
		painter.drawArc(centerX-fillR, centerY-fillR, fillR * 2, fillR * 2, 
			(angle_zero < angle_value ? angle_zero : angle_value) *16, 
			abs(angle_zero - angle_value) *16 );

		// Draw Background Dashboard 1
		for (int i=0; i<4; ++i) {
			if (i == 1 || i == 2) {
				painter.setPen(QPen(bound_color, boundWidth, Qt::SolidLine, Qt::FlatCap));
			} else {
				painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
			}

			drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(bounds[i], bounds[0], bounds[3], beginAngle, endAngle));
		}
		painter.setPen(QPen(fill_color, 1, Qt::SolidLine, Qt::FlatCap));
		drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(0, bounds[0], bounds[3], beginAngle, endAngle));


		// Draw Background Dashboard 2
		painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
		int angle_starts = beginAngle < endAngle ? beginAngle : endAngle;
		int angle_length = abs(beginAngle - endAngle);
		painter.drawArc(centerX-scaleInnerR, centerY-scaleInnerR, scaleInnerR*2, scaleInnerR*2, angle_starts*16-1, angle_length*16);
		painter.drawArc(centerX-scaleOuterR, centerY-scaleOuterR, scaleOuterR*2, scaleOuterR*2, angle_starts*16-1, angle_length*16);
	}


	void drawTextBar(QPainter& painter, float value, int centerX, int centerY, int width, int height, QString name,
		QColor& fg_color, QColor& fill_color) 
	{
		int edgeWidth = width * 0.007;
		int textSize = height * 0.7;

		painter.setPen(QPen(fill_color, height, Qt::SolidLine, Qt::FlatCap));
		painter.drawLine(centerX-width/2, centerY, centerX-width/2+width*value, centerY);

		painter.setPen(QPen(fg_color, edgeWidth, Qt::SolidLine));
		painter.drawRect(centerX-width/2, centerY-height/2, width, height);

		QFont font = painter.font();
		font.setPointSize(textSize);
		painter.setFont(font);
		painter.drawText(
			centerX-width/2, centerY-height/2, width, height, Qt::AlignCenter | Qt::AlignVCenter, name
			);
	}


	void drawSteeringWheel(QPainter& painter, float steering_wheel_angle, int centerX, int centerY, int width,
		QImage& wheel_image, QColor fg_color, QColor fill_color)
	{
		int radius = width * 0.66;
		int beginAngle = 200;
		int endAngle = -20;
		float minValue = -9.42;
		float maxValue = 9.42;
		int scaleWidth = width * 0.014;
		int scaleInnerR = width / 2 - 14 * width / 128;
		int scaleOuterR = width / 2 - 2 * width / 128;
		int fillR = (scaleInnerR + scaleOuterR) / 2;
		int fillWidth = scaleOuterR - scaleInnerR;
		int textYOffset = width * 0.38;
		int textSize = width * 0.07;

		// draw steering_angle_value
		int centerDegree = getDegree(0, minValue, maxValue, beginAngle, endAngle);
		int valueDegree = getDegree(steering_wheel_angle, minValue, maxValue, beginAngle, endAngle);

		painter.setPen(QPen(fill_color, fillWidth, Qt::SolidLine, Qt::FlatCap));
		painter.drawArc(centerX-fillR, centerY-fillR, fillR*2, fillR*2, (centerDegree < valueDegree ? centerDegree : valueDegree)*16, abs(centerDegree-valueDegree)*16);


		// draw dashboard
		painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine, Qt::FlatCap));
		painter.drawArc(centerX-scaleOuterR, centerY-scaleOuterR, scaleOuterR*2, scaleOuterR*2, endAngle*16-1, (beginAngle-endAngle)*16);

		drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(6.28, minValue, maxValue, beginAngle, endAngle));
		drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(-6.28, minValue, maxValue, beginAngle, endAngle));

		painter.setPen(QPen(fill_color, 1, Qt::SolidLine, Qt::FlatCap));
		drawRadiusLine(painter, centerX, centerY, scaleInnerR, scaleOuterR, getDegree(0, minValue, maxValue, beginAngle, endAngle));

		// draw wheel
		painter.translate(centerX, centerY);
		painter.rotate(steering_wheel_angle * 180 / 3.14);
		painter.drawImage(QRect(-radius/2, -radius/2, radius, radius), wheel_image);
		painter.resetTransform();

		// draw degree
		painter.setPen(QPen(fg_color, scaleWidth, Qt::SolidLine));
		QFont font = painter.font();
		font.setPointSize(textSize);
		font.setBold(true);
		painter.setFont(font);
		QString degreeString;
		float tmpDegree = steering_wheel_angle * 180 / 3.14;
		if (tmpDegree < 1) {
			degreeString = QString::number(tmpDegree, 'f', 2) + "°";
		} else {
			degreeString = QString::number(tmpDegree, 'g', 3) + "°";
		}
		painter.drawText(
			centerX - width/2, centerY + textYOffset, width, textSize * 1.1,
			Qt::AlignCenter | Qt::AlignVCenter, degreeString
			);
	}

	void drawTurnSignal(QPainter& painter,uint8_t turn_signal, int centerX, int centerY, int width, int height, QImage& signal_on_image, QImage& signal_off_image) {
		
		painter.translate(centerX - (width/2 - height/2), centerY);
		painter.rotate(180);
		painter.drawImage(QRect(-height/2, -height/2, height, height), 
			turn_signal == 1 ? signal_on_image : signal_off_image);
		painter.resetTransform();

		painter.translate(centerX + (width/2 - height/2), centerY);
		painter.drawImage(QRect(-height/2, -height/2, height, height),
			turn_signal == 2 ? signal_on_image : signal_off_image);
		painter.resetTransform();
		
	}


	void DashboardDisplay::draw() {
		ScopedPixelBuffer buffer = overlay_->getBuffer();
		QImage Hud = buffer.getQImage(*overlay_, bg_color_);
		QPainter painter( &Hud );
		painter.setRenderHint(QPainter::Antialiasing, true);
		painter.setRenderHint(QPainter::SmoothPixmapTransform, true);


		// Left Dashboard
		QColor pointer_color(220, 203, 214, fg_color_.alpha());
		drawSpeed(painter, data_.speed, height_/2, height_/2, height_ * 0.75, fg_color_, pointer_color);

		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_.y_accel_lower_bound, data_.y_accel_lower_comfort, data_.y_accel_upper_comfort, data_.y_accel_upper_bound};
			drawBoundedBar(painter, data_.y_accel_data, bounds,
				height_/2, height_/2, height_, 220, 120, fg_color_, bound_color, fill_color);
		}

		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_.y_jerk_lower_bound, data_.y_jerk_lower_comfort, data_.y_jerk_upper_comfort, data_.y_jerk_upper_bound};
			drawBoundedBar(painter, data_.y_jerk_data, bounds,
				height_/2, height_/2, height_, -40, 60, fg_color_, bound_color, fill_color);
		}

		{
			QColor fill_color(100, 100, 255, fg_color_.alpha());
			drawTextBar(painter, data_.throttle_percent, height_/2, height_*0.75, height_* 0.5, height_* 0.08,
				"Throttle", fg_color_, fill_color);
		}

		{
			QColor fill_color(255, 100, 100, fg_color_.alpha());
			drawTextBar(painter, data_.brake_percent, height_/2, height_*0.85, height_* 0.5, height_* 0.08,
				"Brake", fg_color_, fill_color);
		}

		// Right Dashboard
		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_.x_accel_lower_bound, data_.x_accel_lower_comfort, data_.x_accel_upper_comfort, data_.x_accel_upper_bound};
			drawBoundedBar(painter, data_.x_accel_data, bounds,
				width_-height_/2, height_/2 + height_*0.01, height_, 140, 40, fg_color_, bound_color, fill_color);
		}

		{
			QColor bound_color(255, 50, 50, fg_color_.alpha());
			QColor fill_color(220, 203, 214, fg_color_.alpha());
			float bounds[4] = {data_.x_jerk_lower_bound, data_.x_jerk_lower_comfort, data_.x_jerk_upper_comfort, data_.x_jerk_upper_bound};
			drawBoundedBar(painter, data_.x_jerk_data, bounds,
				width_-height_/2, height_/2 - height_*0.05, height_, -140, -40, fg_color_, bound_color, fill_color);
		}

		{
			QColor fill_color(100, 100, 100, fg_color_.alpha());
			drawSteeringWheel(painter, data_.steering_wheel_angle, width_-height_/2, height_/2, height_ * 0.7, 
				wheel_image_, fg_color_, fill_color);
		}

		drawTurnSignal(painter, data_.turn_signal, width_-height_/2, height_*0.3, height_, height_*0.15, signal_on_image_, signal_off_image_);

		
		painter.end();
	}



	void DashboardDisplay::updateSize() {
		int size = size_property_->getInt();
		if (size <= 32 || size >= 1024) {
			return;
		}
		width_ = 2.1 * size_property_->getInt();
		height_ = size_property_->getInt();
		update_required_ = true;
	}

	void DashboardDisplay::updateTopic() {
		unsubscribe();
		subscribe();
	}

	void updateImageColor(QImage& img, QColor& color) {
		for(int i=0; i<img.width(); i++){
			for(int j=0; j<img.height(); j++) {
				if (qAlpha(img.pixel(i,j)) > 0.01)
					img.setPixel(i,j,color.rgba());
			}
		}
	}

	void DashboardDisplay::updateFGAlpha() {
		OverlayDisplay::updateFGAlpha();

		QColor signal_off_color(0,0,0, fg_color_.alpha());
		updateImageColor(wheel_image_, fg_color_);
		updateImageColor(signal_on_image_, fg_color_);
		updateImageColor(signal_off_image_, signal_off_color);
	}
}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::DashboardDisplay,rviz::Display )
// END_TUTORIAL