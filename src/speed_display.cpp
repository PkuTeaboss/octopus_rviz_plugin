// Cheng Zhang

#include "speed_display.h"
#include <rviz/uniform_string_stream.h>

#include <QPainter>

#include <iomanip>

namespace octopus_rviz_plugin
{


  SpeedDisplay::SpeedDisplay()
  : rviz::Display(), update_required_(false), data_(0)
  {
    topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      //ros::message_traits::datatype<std_msgs::Float32>(),
      ros::message_traits::datatype<dbw_mkz_msgs::TwistCmd>(),
      "dbw_mkz_msgs::TwistCmd topic to subscribe to.",
      this, SLOT( updateTopic() ));

    size_property_ = new rviz::IntProperty(
      "size", 200, "", this, SLOT(updateSize())
      );
    left_property_ = new rviz::IntProperty(
      "left", 10, "", this, SLOT(updateLeft())
      );
    top_property_ = new rviz::IntProperty(
      "top", 10, "", this, SLOT(updateTop())
      );

    pointer_color_property_ = new rviz::ColorProperty(
      "Pointer Color", QColor(220, 203, 214), "", this, SLOT(updatePointerColor())
      );

    fg_color_property_ = new rviz::ColorProperty(
      "Dashboard Color", QColor(220, 220, 0), "", this, SLOT(updateFGColor())
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

  }


  SpeedDisplay::~SpeedDisplay(){
  }


  void SpeedDisplay::onInitialize() {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "SpeedDisplay" << count++;
    overlay_.reset(new OverlayObject(ss.str()));

    onEnable();
    updateSize();
    updateLeft();
    updateTop();
    updatePointerColor();
    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();

    overlay_->updateTextureSize(width_, height_);
    overlay_->setPosition(left_, top_);
    overlay_->setDimensions(overlay_->getTextureWidth(),
      overlay_->getTextureHeight());
    overlay_->show(); 
    draw(data_);
  }


  void SpeedDisplay::update(float wall_dt, float ros_dt) {
    if (update_required_) {
      update_required_ = false;
      overlay_->updateTextureSize(width_, height_);
      overlay_->setPosition(left_, top_);
      overlay_->setDimensions(overlay_->getTextureWidth(),
        overlay_->getTextureHeight());
      draw(data_);
    }
  }

  void SpeedDisplay::subscribe() {
    std::string topic_name = topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      ros::NodeHandle n;
      sub_ = n.subscribe(topic_name, 1, &SpeedDisplay::processMessage, this);
    }
  }

  void SpeedDisplay::unsubscribe() {
    sub_.shutdown();
  }

  void SpeedDisplay::onEnable() {
    subscribe();
    overlay_->show();
  }

  void SpeedDisplay::onDisable() {
    unsubscribe();
    overlay_->hide();
  }

  /*
  void SpeedDisplay::processMessage(const std_msgs::Float32::ConstPtr& msg) {
    if (!overlay_->isVisible()) {
      return;
    }
    if (data_ != msg->data) {
      data_ = msg->data;
      update_required_ = true;
    }
  }*/

  void SpeedDisplay::processMessage(const dbw_mkz_msgs::TwistCmd::ConstPtr& msg) {
    if (!overlay_->isVisible()) {
      return;
    }
    if (data_ != msg->twist.linear.x) {
      data_ = msg->twist.linear.x;
      update_required_ = true;
    }
  }


  void SpeedDisplay::draw(double val) {

    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color_);
    QPainter painter( &Hud );
    painter.setRenderHint(QPainter::Antialiasing, true);

    int max_speed = 140;
    int interval = 5;
    int textCount = 4;
    float min_degree = 60;
    float max_degree = 300;

    int centerX = width_ / 2;
    int centerY = height_ / 2;
    int scaleInnerR1 = width_ / 2 - 9 * width_ / 128;
    int scaleInnerR2 = width_ / 2 - 6 * width_ / 128;
    int scaleOuterR = width_ / 2 - 2 * width_ / 128;
    int scaleWidth = 3 * width_ / 200;
    int textR = width_ / 2 - 19 * width_ / 128;
    int textSize = 8 * width_ / 128;
    int speedSize = 14 * width_ / 128;
    int pointerInnerR = width_ / 2 - 40 * width_ / 128;
    int pointerOuterR = width_ / 2 - 10;
    int pointerWidth = 2 * width_ / 200;


    int n = max_speed / interval;
    float interval_degree = (max_degree - min_degree) / n;

    // Draw BackGround Dash Board
    painter.setPen(QPen(fg_color_, scaleWidth, Qt::SolidLine));
    QFont font = painter.font();
    font.setPointSize(textSize);
    font.setBold(true);
    painter.setFont(font);

    for (int i=0; i <= n; ++i) {
      float degree = min_degree + interval_degree * i;
      int scaleInnerX ,scaleInnerY;
      int scaleOuterX = centerX + scaleOuterR * (-sin(degree * 3.14 / 180));
      int scaleOuterY = centerY + scaleOuterR * (cos(degree * 3.14 / 180));
      if (i % textCount == 0) {
        scaleInnerX = centerX + scaleInnerR1 * (-sin(degree * 3.14 / 180));
        scaleInnerY = centerY + scaleInnerR1 * (cos(degree * 3.14 / 180));

        int textX = centerX + textR * (-sin(degree * 3.14 / 180));
        int textY = centerY + textR * (cos(degree * 3.14 / 180));

        std::stringstream ss;
        ss << (interval * i);

        painter.drawText(
          textX - textSize * 1.3, textY - textSize/2, textSize*2.5, textSize,
          Qt::AlignCenter | Qt::AlignVCenter, ss.str().c_str());
      } else {
        scaleInnerX = centerX + scaleInnerR2 * (-sin(degree * 3.14 / 180));
        scaleInnerY = centerY + scaleInnerR2 * (cos(degree * 3.14 / 180));
      }

      painter.drawLine(scaleInnerX, scaleInnerY, scaleOuterX, scaleOuterY);
    }

    // Draw Speed
    painter.drawText(
      centerX - textSize * 3, centerY + speedSize * 2, textSize*6, textSize,
      Qt::AlignCenter | Qt::AlignVCenter, "MPH"
      );

    font.setPointSize(speedSize);
    painter.setFont(font);

    std::stringstream ss;
    ss<< std::fixed;
    ss<< std::setprecision(1);
    ss<< val;
    painter.drawText(
      centerX - speedSize * 3, centerY - speedSize/2, speedSize*6, speedSize,
      Qt::AlignCenter | Qt::AlignVCenter, ss.str().c_str()
      );



    // Draw Pointer
    if (val < 0) val = 0;
    else if (val > max_speed) val = max_speed;

    painter.setPen(QPen(pointer_color_, pointerWidth, Qt::SolidLine));
    int degree = min_degree + round(val / max_speed * (max_degree - min_degree));
    int pointerInnerX = centerX + pointerInnerR * (-sin(degree * 3.14 / 180));
    int pointerInnerY = centerY + pointerInnerR * (cos(degree * 3.14 / 180));
    int pointerOuterX = centerX + pointerOuterR * (-sin(degree * 3.14 / 180));
    int pointerOuterY = centerY + pointerOuterR * (cos(degree * 3.14 / 180));
    painter.drawLine(pointerInnerX, pointerInnerY, pointerOuterX, pointerOuterY);
    

    painter.end();

  }


  void SpeedDisplay::updateTopic() {
    unsubscribe();
    subscribe();
  }


  void SpeedDisplay::updateSize() {
    int size = size_property_->getInt();
    if (size <= 32 || size >= 1024) {
      return;
    }
    width_ = size_property_->getInt();
    height_ = size_property_->getInt();
    update_required_ = true;
  }

  void SpeedDisplay::updateLeft() {
    left_ = left_property_->getInt();
    update_required_ = true;
  }

  void SpeedDisplay::updateTop() {
    top_ = top_property_->getInt();
    update_required_ = true;
  }

  void SpeedDisplay::updatePointerColor() {
    int alpha = pointer_color_.alpha();
    pointer_color_ = pointer_color_property_->getColor();
    pointer_color_.setAlpha(alpha);
    update_required_ = true;
  }


  void SpeedDisplay::updateFGColor() {
    int alpha = fg_color_.alpha();
    fg_color_ = fg_color_property_->getColor();
    fg_color_.setAlpha(alpha);
    update_required_ = true;
  }

  void SpeedDisplay::updateFGAlpha() {
    pointer_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
    update_required_ = true;
  }

  void SpeedDisplay::updateBGColor() {
    int alpha = bg_color_.alpha();
    bg_color_ = bg_color_property_->getColor();
    bg_color_.setAlpha(alpha);
    update_required_ = true;
  }

  void SpeedDisplay::updateBGAlpha() {
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
    update_required_ = true;
  }


  bool SpeedDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + height_ > y &&
      left_ < x && left_ + width_ > x);
  }

  void SpeedDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
    update_required_ = true;
  }

  void SpeedDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
    update_required_ = true;
  }

}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::SpeedDisplay,rviz::Display )
// END_TUTORIAL
