// Cheng Zhang

#include "float_bounded_display.h"

#include <QPainter>

namespace octopus_rviz_plugin
{
	FloatBoundedDisplay::FloatBoundedDisplay()
	: OverlayDisplay()
	{
		topic_property_ = new rviz::RosTopicProperty(
			"Topic", "",
			ros::message_traits::datatype<std_msgs::Float32>(),
			"std_msgs::Float32 topic to subscribe to.",
			this, SLOT( updateTopic() ));

		name_property_ = new rviz::StringProperty(
			"Variable Name", "Undefined", "", this,SLOT(updateName()) 
			);

		max_value_property_ = new rviz::FloatProperty(
			"Max Value", 1, "", this,SLOT(updateMaxValue())
			);
		upper_bound_property_ = new rviz::FloatProperty(
			"Upper Bound", 0.6, "", this,SLOT(updateUpperBound())
			);
		lower_bound_property_ = new rviz::FloatProperty(
			"Lower Bound", -0.8, "", this,SLOT(updateLowerBound())
			);
		min_value_property_ = new rviz::FloatProperty(
			"Min Value", -1, "", this,SLOT(updateMinValue())
			);

		OverlayDisplay::initProperties();

		pointer_color_property_ = new rviz::ColorProperty(
			"Pointer Color", QColor(220, 203, 214), "", this, SLOT(updatePointerColor())
			);

		bound_color_property_ = new rviz::ColorProperty(
			"Bound Color", QColor(255, 50, 50), "", this, SLOT(updateBoundColor())
			);

		data_ = 0;
	}

	FloatBoundedDisplay::~FloatBoundedDisplay() {

	}

	void FloatBoundedDisplay::onInitialize() {
		OverlayDisplay::onInitialize();

		updateName();
		updateMaxValue();
		updateUpperBound();
		updateLowerBound();
		updateMinValue();
		updatePointerColor();
		updateBoundColor();
	}

	void FloatBoundedDisplay::update(float wall_dt, float ros_dt) {
		OverlayDisplay::update(wall_dt, ros_dt);
	}

	void FloatBoundedDisplay::onEnable() {
		OverlayDisplay::onEnable();
	}

	void FloatBoundedDisplay::onDisable() {
		OverlayDisplay::onDisable();
	}

	void FloatBoundedDisplay::processMessage(const std_msgs::Float32::ConstPtr& msg) {
		if (!overlay_->isVisible()) {
			return;
		}
		if (data_ != msg->data) {
			data_ = msg->data;
			update_required_ = true;
		}
	}

	void FloatBoundedDisplay::subscribe() {
		std::string topic_name = topic_property_->getTopicStd();
		if (topic_name.length() > 0 && topic_name != "/") {
			ros::NodeHandle n;
			sub_ = n.subscribe(topic_name, 1, &FloatBoundedDisplay::processMessage, this);
		}
	}

	void FloatBoundedDisplay::unsubscribe() {
		sub_.shutdown();
	}


	void FloatBoundedDisplay::draw() {
		ScopedPixelBuffer buffer = overlay_->getBuffer();
		QImage Hud = buffer.getQImage(*overlay_, bg_color_);
		QPainter painter( &Hud );
		painter.setRenderHint(QPainter::Antialiasing, true);

		if (min_value_ <= lower_bound_ && lower_bound_ < upper_bound_ && upper_bound_ <= max_value_) {
			int arc_angle = 25;
			int centerX = width_ / 2;
			int scaleR = height_ / (sin(arc_angle * 3.14 / 180) + 0.03); 	// sin(15 degree)
			int centerY = height_ * 0.1 + scaleR;
			int min_angle = 90 - arc_angle;
			int max_angle = 90 + arc_angle;

			// Draw BackGround Dash Board
			int scaleEndWidth = height_ * 0.06;
			int scaleBoundWidth = height_* 0.09;
			int scaleWidth = 3 * height_ / 100;
			int scaleTextSize = 15 * height_ / 100;
			int scaleTextR = scaleR - height_ * 0.23;

			painter.setPen(QPen(fg_color_, scaleWidth, Qt::SolidLine, Qt::FlatCap));
			painter.drawArc(-scaleR + centerX, centerY - scaleR, 2 * scaleR, 2 * scaleR, min_angle*16, 2*arc_angle*16);


			float a[4] = {min_angle, 0, 0, max_angle};
			a[1] = min_angle + (lower_bound_ - min_value_) / (max_value_ - min_value_) * (max_angle - min_angle);
			a[2] = min_angle + (upper_bound_ - min_value_) / (max_value_ - min_value_) * (max_angle - min_angle);
			for (int i=0; i<4; ++i) {
				int scaleInnerR, scaleOuterR;
				if (i == 1 || i == 2) {
					painter.setPen(QPen(bound_color_, scaleWidth, Qt::SolidLine));
					QFont font = painter.font();
					font.setPointSize(scaleTextSize);
					font.setBold(true);
					painter.setFont(font);
					scaleInnerR = scaleR - scaleBoundWidth;
					scaleOuterR = scaleR + scaleBoundWidth;

					int textX = centerX - scaleTextR * ( cos(a[i] * 3.14 / 180));
					int textY = centerY - scaleTextR * ( sin(a[i] * 3.14 / 180));
					float tmp_bound = i==1 ? lower_bound_ : upper_bound_ ;
					painter.drawText(
						textX - scaleTextSize * 3, textY - scaleTextSize/2, scaleTextSize*6, scaleTextSize,
						Qt::AlignCenter | Qt::AlignVCenter, QString::number(tmp_bound, 'g', 4)
						);
				} else {
					painter.setPen(QPen(fg_color_, scaleWidth, Qt::SolidLine));
					scaleInnerR = scaleR - scaleEndWidth;
					scaleOuterR = scaleR + scaleEndWidth;
				}
				int scaleInnerX = centerX - scaleInnerR * ( cos(a[i] * 3.14 / 180));
				int scaleInnerY = centerY - scaleInnerR * ( sin(a[i] * 3.14 / 180));
				int scaleOuterX = centerX - scaleOuterR * ( cos(a[i] * 3.14 / 180));
				int scaleOuterY = centerY - scaleOuterR * ( sin(a[i] * 3.14 / 180));
				painter.drawLine(scaleInnerX, scaleInnerY, scaleOuterX, scaleOuterY);
			}


			// Draw Data Value
			int dataTextTop = height_ * 0.5;
			int dataTextSize = height_ * 0.2;
			painter.setPen(QPen(fg_color_, scaleWidth, Qt::SolidLine));
			QFont font = painter.font();
			font.setPointSize(dataTextSize);
			painter.setFont(font);
			painter.drawText(
				0, dataTextTop, width_, dataTextSize, Qt::AlignCenter | Qt::AlignVCenter, QString::number(data_, 'g', 4)
				);

			// Draw Variable Name
			int nameTextSize = height_ * 0.15;
			font = painter.font();
			font.setPointSize(nameTextSize);
			painter.setFont(font);
			painter.drawText(
				0, height_ - dataTextSize, width_, dataTextSize, Qt::AlignCenter | Qt::AlignVCenter, name_
				);


			// Draw Pointer
			int pointerInnerR = scaleR - height_ * 0.18;
			int pointerOuterR = scaleR + height_ * 0.12;
			int pointerWidth = 2 * height_ / 100;
			int tmpData = data_;
			if (tmpData < min_value_) tmpData = min_value_;
			else if (tmpData > max_value_) tmpData = max_value_;
			int angle = min_angle + (tmpData - min_value_) / (max_value_ - min_value_) * (max_angle - min_angle);
			int pointerInnerX = centerX - pointerInnerR * ( cos(angle * 3.14 / 180));
			int pointerInnerY = centerY - pointerInnerR * ( sin(angle * 3.14 / 180));
			int pointerOuterX = centerX - pointerOuterR * ( cos(angle * 3.14 / 180));
			int pointerOuterY = centerY - pointerOuterR * ( sin(angle * 3.14 / 180));
			painter.setPen(QPen(pointer_color_, scaleWidth, Qt::SolidLine));
			painter.drawLine(pointerInnerX, pointerInnerY, pointerOuterX, pointerOuterY);
		}

		painter.end();
	}


	void FloatBoundedDisplay::updateSize() {
		int size = size_property_->getInt();
		if (size <= 32 || size >= 1024) {
			return;
		}
		width_ = 2 * size_property_->getInt();
		height_ = size_property_->getInt();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateTopic() {
		unsubscribe();
		subscribe();
	}

	void FloatBoundedDisplay::updateName() {
		name_ = name_property_ -> getString();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateMaxValue() {
		max_value_ = max_value_property_->getFloat();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateUpperBound() {
		upper_bound_ = upper_bound_property_->getFloat();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateLowerBound() {
		lower_bound_ = lower_bound_property_->getFloat();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateMinValue() {
		min_value_ = min_value_property_->getFloat();
		update_required_ = true;
	}

	void FloatBoundedDisplay::updatePointerColor() {
		int alpha = pointer_color_.alpha();
		pointer_color_ = pointer_color_property_->getColor();
		pointer_color_.setAlpha(alpha);
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateBoundColor() {
		int alpha = bound_color_.alpha();
		bound_color_ = bound_color_property_->getColor();
		bound_color_.setAlpha(alpha);
		update_required_ = true;
	}

	void FloatBoundedDisplay::updateFGAlpha() {
		OverlayDisplay::updateFGAlpha();
		bound_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
		pointer_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
	}
}

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::FloatBoundedDisplay,rviz::Display )
// END_TUTORIAL