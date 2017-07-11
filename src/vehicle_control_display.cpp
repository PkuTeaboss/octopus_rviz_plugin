// Cheng Zhang

#include "vehicle_control_display.h"
#include <rviz/uniform_string_stream.h>

#include <QPainter>

#include <iomanip>
#include <ros/package.h>

void updateColor(QColor& color, rviz::ColorProperty* color_property) {
    int alpha = color.alpha();
    color = color_property->getColor();
    color.setAlpha(alpha);
}

void updateImageColor(QImage& img, QColor& color) {
    for(int i=0; i<img.width(); i++){
        for(int j=0; j<img.height(); j++) {
            if (qAlpha(img.pixel(i,j)) > 0.01)
                img.setPixel(i,j,color.rgba());
        }
    }
}

namespace octopus_rviz_plugin
{

    VehicleControlDisplay::VehicleControlDisplay()
    : rviz::Display(), update_required_(false)
    {
        steering_topic_property_ = new rviz::RosTopicProperty(
          "SteeringReport Topic", "",
          ros::message_traits::datatype<dbw_mkz_msgs::SteeringReport>(),
          "dbw_mkz_msgs::SteeringReport topic to subscribe to.",
          this, SLOT( updateSteeringTopic() ));

        brake_topic_property_ = new rviz::RosTopicProperty(
          "BrakeReport Topic", "",
          ros::message_traits::datatype<dbw_mkz_msgs::BrakeReport>(),
          "dbw_mkz_msgs::BrakeReport topic to subscribe to.",
          this, SLOT( updateBrakeTopic() ));

        throttle_topic_property_ = new rviz::RosTopicProperty(
          "Throttle Topic", "",
          ros::message_traits::datatype<dbw_mkz_msgs::ThrottleReport>(),
          "dbw_mkz_msgs::ThrottleReport topic to subscribe to.",
          this, SLOT( updateThrottleTopic() ));

        signal_topic_property_ = new rviz::RosTopicProperty(
          "TurnSignal Topic", "",
          ros::message_traits::datatype<dbw_mkz_msgs::TurnSignal>(),
          "dbw_mkz_msgs::TurnSignal topic to subscribe to.",
          this, SLOT( updateSignalTopic() ));

        size_property_ = new rviz::IntProperty(
          "size", 200, "", this, SLOT(updateSize())
          );
        left_property_ = new rviz::IntProperty(
            "left", 20, "", this, SLOT(updateLeft())
            );
        top_property_ = new rviz::IntProperty(
          "top", 20, "", this, SLOT(updateTop())
          );

        fg_color_property_ = new rviz::ColorProperty(
          "Dashboard Color", QColor(220, 220, 0), "", this, SLOT(updateFGColor())
          );

        throttle_color_property_ = new rviz::ColorProperty(
          "Throttle Color", QColor(100, 100, 255), "", this, SLOT(updateThrottleColor())
          );

        brake_color_property_ = new rviz::ColorProperty(
          "Brake Color", QColor(255, 100, 100), "", this, SLOT(updateBrakeColor())
          );

        signal_color_property_ = new rviz::ColorProperty(
            "Signal Color", QColor(220, 220, 100), "", this, SLOT(updateSignalColor())
            );

        fg_alpha_property_ = new rviz::FloatProperty(
          "Foreground Alpha", 0.8, "", this,SLOT(updateFGAlpha())
          );

        bg_color_property_ = new rviz::ColorProperty(
          "Background Color", QColor(0, 0, 0), "", this, SLOT(updateBGColor())
          );
        bg_alpha_property_ = new rviz::FloatProperty(
          "Background Alpha", 0.2, "", this, SLOT(updateBGAlpha())
          );

        wheel_angle_ = 0.0f;
        throttle_angle_ = 32.2f;
        brake_angle_ = 50.0f;
        signal_ = 1;
    }


    VehicleControlDisplay::~VehicleControlDisplay(){
    }


    void VehicleControlDisplay::onInitialize() {
        static int count = 0;
        rviz::UniformStringStream ss;
        ss << "VehicleControlDisplay" << count++;
        overlay_.reset(new OverlayObject(ss.str()));

        wheel_image_  = QImage(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/wheel.png");
        signal_off_image_ = QImage(QString(ros::package::getPath("octopus_rviz_plugin").c_str())+"/media/arrow.png");
        signal_off_color_ = QColor(0, 0, 0, 0.8);
        signal_on_image_ = signal_off_image_;

        onEnable();
        updateSize();
        updateLeft();
        updateTop();
        updateFGColor();
        updateThrottleColor();
        updateBrakeColor();
        updateSignalColor();
        updateFGAlpha();
        updateBGColor();
        updateBGAlpha();

        overlay_->updateTextureSize(width_, height_);
        overlay_->setPosition(left_, top_);
        overlay_->setDimensions(overlay_->getTextureWidth(),
            overlay_->getTextureHeight());
        draw();

    }


    void VehicleControlDisplay::update(float wall_dt, float ros_dt) {
        if (update_required_) {
            update_required_ = false;
            overlay_->updateTextureSize(width_, height_);
            overlay_->setPosition(left_, top_);
            overlay_->setDimensions(overlay_->getTextureWidth(),
                overlay_->getTextureHeight());
            draw();
        }
    }



    void VehicleControlDisplay::onEnable() {
        subscribeSteering();
        subscribeBrake();
        subscribeThrottle();
        subscribeSignal();
        overlay_->show();
    }

    void VehicleControlDisplay::onDisable() {
        unsubscribe(steering_topic_property_);
        unsubscribe(brake_topic_property_);
        unsubscribe(throttle_topic_property_);
        unsubscribe(signal_topic_property_);
        overlay_->hide();
    }

    void VehicleControlDisplay::drawPadelBackGround(QPainter& painter, int centerX, int centerY, int startAngle) {
        int size = height_ * 1.00;
        int lineWidth = 3 * height_ / 200;
        int maxAngle = 80;
        int interval = 8;
        int innerSize = height_ * 0.90;
        painter.setPen(QPen(fg_color_, lineWidth, Qt::SolidLine, Qt::FlatCap));
        painter.drawArc(centerX-size/2, centerY-size/2, size, size, startAngle*16, maxAngle*16);

        for(int angle=0; angle <= maxAngle; angle += interval) {
            int innerX = centerX + innerSize/2 * cos( (startAngle+angle)* 3.14 / 180 );
            int innerY = centerY + innerSize/2 * sin( (startAngle+angle)* 3.14 / 180 );
            int outerX = centerX + size/2 * cos( (startAngle+angle)* 3.14 / 180 );
            int outerY = centerY + size/2 * sin( (startAngle+angle)* 3.14 / 180 );
            painter.drawLine(innerX, innerY, outerX, outerY);
        }
    }


    void VehicleControlDisplay::draw() {
        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_, bg_color_);
        QPainter painter( &Hud );
        painter.setRenderHint(QPainter::Antialiasing, true);

        int centerX = width_ * 0.5;
        int centerY = height_ * 0.6;

        int wheelSize = height_ * 0.67;
        // Draw wheel
        //QTransform wheelRotate;
        //wheelRotate.rotate(wheel_angle_);
        painter.translate(centerX, centerY);
        painter.rotate(wheel_angle_);
        painter.drawImage(QRect(-wheelSize/2, -wheelSize/2, wheelSize, wheelSize), 
            wheel_image_);
        painter.resetTransform();

        int padelSize = height_ * 0.88;
        int padelWidth = height_ * 0.12;
        // Draw Throttle and Brake
        painter.setPen(QPen(throttle_color_, padelWidth, Qt::SolidLine, Qt::FlatCap));
        painter.drawArc(centerX-padelSize/2, centerY-padelSize/2, padelSize, padelSize, (220.0f - throttle_angle_)*16, throttle_angle_*16);
        painter.setPen(QPen(brake_color_, padelWidth, Qt::SolidLine, Qt::FlatCap));
        painter.drawArc(centerX-padelSize/2, centerY-padelSize/2, padelSize, padelSize, -40 * 16, brake_angle_*16);

        drawPadelBackGround(painter, centerX, centerY, 140);
        drawPadelBackGround(painter, centerX, centerY, -40);


        int signalSize = height_ * 0.25;
        int signalHeight = height_ * 0.02;
        int signalXOffset = height_ * 0.28;
        // Draw Signal
        if (signal_ == 2) {
            // Right Signal
            painter.drawImage(QRect(centerX+signalXOffset-signalSize/2, signalHeight, signalSize, signalSize),
                signal_on_image_);
        } else {
            painter.drawImage(QRect(centerX+signalXOffset-signalSize/2, signalHeight, signalSize, signalSize),
                signal_off_image_);
        }

        QTransform signalRotate;
        signalRotate.rotate(180);
        if (signal_ == 1) {
            // Left Signal
            painter.drawImage(QRect(centerX-signalXOffset-signalSize/2, signalHeight, signalSize, signalSize),
                signal_on_image_.transformed(signalRotate));
        } else {
            painter.drawImage(QRect(centerX-signalXOffset-signalSize/2, signalHeight, signalSize, signalSize),
                signal_off_image_.transformed(signalRotate));
        }

        painter.end();
    }

    void VehicleControlDisplay::subscribeSteering() {
        std::string topic_name = steering_topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/") {
            ros::NodeHandle n;
            sub_map_[topic_name] = n.subscribe(topic_name, 1, &VehicleControlDisplay::processSteeringMessage, this);
        }
    }

    void VehicleControlDisplay::subscribeBrake() {
        std::string topic_name = brake_topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/") {
            ros::NodeHandle n;
            sub_map_[topic_name] = n.subscribe(topic_name, 1, &VehicleControlDisplay::processBrakeMessage, this);
        }
    }

    void VehicleControlDisplay::subscribeThrottle() {
        std::string topic_name = throttle_topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/") {
            ros::NodeHandle n;
            sub_map_[topic_name] = n.subscribe(topic_name, 1, &VehicleControlDisplay::processThrottleMessage, this);
        }
    }

    void VehicleControlDisplay::subscribeSignal() {
        std::string topic_name = signal_topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/") {
            ros::NodeHandle n;
            sub_map_[topic_name] = n.subscribe(topic_name, 1, &VehicleControlDisplay::processSignalMessage, this);
        }
    }



    void VehicleControlDisplay::unsubscribe(rviz::RosTopicProperty* topic_property) {
        std::string topic_name = topic_property->getTopicStd();
        if (sub_map_.find(topic_name) != sub_map_.end()) {
            sub_map_[topic_name].shutdown();
        }
    }

    void VehicleControlDisplay::processSteeringMessage(const dbw_mkz_msgs::SteeringReport::ConstPtr& msg) {
        if (!overlay_->isVisible()) {
            return;
        }
        if (wheel_angle_ != msg->steering_wheel_angle * 180 / 3.14 ){
            wheel_angle_ = msg->steering_wheel_angle * 180 / 3.14 ;
            update_required_ = true;
        }
    }

    void VehicleControlDisplay::processBrakeMessage(const dbw_mkz_msgs::BrakeReport::ConstPtr& msg) {
        if (!overlay_->isVisible()) {
            return;
        }
        float tmp = (msg->pedal_input - 0.15) / 0.35 * 80;
        if (brake_angle_ != tmp) {
            brake_angle_ = tmp;
            update_required_ = true;
        }
    }

    void VehicleControlDisplay::processThrottleMessage(const dbw_mkz_msgs::ThrottleReport::ConstPtr& msg) {
        if (!overlay_->isVisible()) {
            return;
        }
        float tmp = (msg->pedal_input - 0.15) / 0.35 * 80;
        if (throttle_angle_ != tmp) {
            throttle_angle_ = tmp;
            update_required_ = true;
        }
    }

    void VehicleControlDisplay::processSignalMessage(const dbw_mkz_msgs::TurnSignal::ConstPtr& msg) {
        if (!overlay_->isVisible()) {
            return;
        }
        if (signal_ != msg->value) {
            signal_ = msg->value;
            update_required_ = true;
        }
    }


    void VehicleControlDisplay::updateSteeringTopic() {
        unsubscribe(steering_topic_property_);
        subscribeSteering();
    }

    void VehicleControlDisplay::updateBrakeTopic() {
        unsubscribe(brake_topic_property_);
        subscribeBrake();
    }

    void VehicleControlDisplay::updateThrottleTopic() {
        unsubscribe(throttle_topic_property_);
        subscribeThrottle();
    }

    void VehicleControlDisplay::updateSignalTopic() {
        unsubscribe(signal_topic_property_);
        subscribeSignal();
    }




    void VehicleControlDisplay::updateSize() {
        int size = size_property_->getInt();
        if (size <= 32 || size >= 1024) {
            return;
        }
        width_ = size * 1.3;
        height_ = size;
        update_required_ = true;
    }

    void VehicleControlDisplay::updateLeft() {
        left_ = left_property_->getInt();
        update_required_ = true;
    }

    void VehicleControlDisplay::updateTop() {
        top_ = top_property_->getInt();
        update_required_ = true;
    }


    void VehicleControlDisplay::updateFGColor() {
        updateColor(fg_color_, fg_color_property_);

        updateImageColor(wheel_image_, fg_color_);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateThrottleColor() {
        updateColor(throttle_color_, throttle_color_property_);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateBrakeColor() {
        updateColor(brake_color_, brake_color_property_);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateSignalColor() {
        updateColor(signal_on_color_, signal_color_property_);

        updateImageColor(signal_on_image_, signal_on_color_);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateFGAlpha() {
        fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
        throttle_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
        brake_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
        signal_on_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
        signal_off_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);

        updateImageColor(wheel_image_, fg_color_);
        updateImageColor(signal_on_image_, signal_on_color_);
        updateImageColor(signal_off_image_, signal_off_color_);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateBGColor() {
        int alpha = bg_color_.alpha();
        bg_color_ = bg_color_property_->getColor();
        bg_color_.setAlpha(alpha);
        update_required_ = true;
    }

    void VehicleControlDisplay::updateBGAlpha() {
        bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
        update_required_ = true;
    }


    bool VehicleControlDisplay::isInRegion(int x, int y)
    {
        return (top_ < y && top_ + height_ > y &&
          left_ < x && left_ + width_ > x);
    }

    void VehicleControlDisplay::movePosition(int x, int y)
    {
        top_ = y;
        left_ = x;
        update_required_ = true;
    }

    void VehicleControlDisplay::setPosition(int x, int y)
    {
        top_property_->setValue(y);
        left_property_->setValue(x);
        update_required_ = true;
    }

}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::VehicleControlDisplay,rviz::Display )
// END_TUTORIAL
