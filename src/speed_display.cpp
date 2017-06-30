// Cheng Zhang

#include "speed_display.h"
#include <rviz/uniform_string_stream.h>

#include <QPainter>

namespace octopus_rviz_plugin
{


  SpeedDisplay::SpeedDisplay()
  : rviz::Display(), width_(128), height_(128), left_(16), top_(16)
  {
    fg_color_property_ = new rviz::ColorProperty(
      "Foreground Color", QColor(220, 220, 0), "", this, SLOT(updateFGColor())
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

    overlay_->updateTextureSize(width_, height_);
    overlay_->setPosition(left_, top_);
    overlay_->setDimensions(overlay_->getTextureWidth(),
      overlay_->getTextureHeight());

    overlay_->show(); 

    updateFGColor();
    updateFGAlpha();
    updateBGColor();
    updateBGAlpha();
  }


  void SpeedDisplay::update(float wall_dt, float ros_dt) {

    draw(20);

  }

  void SpeedDisplay::onEnable() {
    overlay_->show();
  }

  void SpeedDisplay::onDisable() {
    overlay_->hide();
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
    int scaleInnerR1 = width_ / 2 - 9;
    int scaleInnerR2 = width_ / 2 - 6;
    int scaleOuterR = width_ / 2 - 2;
    int textR = width_ / 2 - 19;
    int textSize = 8;
    int pointerInnerR = width_ / 2 - 40;
    int pointerOuterR = width_ / 2 - 10;


    int n = max_speed / interval;
    float interval_degree = (max_degree - min_degree) / n;

    // Draw BackGround Dash Board
    painter.setPen(QPen(fg_color_, 3, Qt::SolidLine));
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

    // Draw Pointer
    painter.setPen(QPen(fg_color_, 2, Qt::SolidLine));
    int degree = min_degree + round(val / max_speed * (max_degree - min_degree));
    int pointerInnerX = centerX + pointerInnerR * (-sin(degree * 3.14 / 180));
    int pointerInnerY = centerY + pointerInnerR * (cos(degree * 3.14 / 180));
    int pointerOuterX = centerX + pointerOuterR * (-sin(degree * 3.14 / 180));
    int pointerOuterY = centerY + pointerOuterR * (cos(degree * 3.14 / 180));
    painter.drawLine(pointerInnerX, pointerInnerY, pointerOuterX, pointerOuterY);
    

    painter.end();

  }


  void SpeedDisplay::updateFGColor() {
    int alpha = fg_color_.alpha();
    fg_color_ = fg_color_property_->getColor();
    fg_color_.setAlpha(alpha);
  }

  void SpeedDisplay::updateFGAlpha() {
    fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
  }

  void SpeedDisplay::updateBGColor() {
    int alpha = bg_color_.alpha();
    bg_color_ = bg_color_property_->getColor();
    bg_color_.setAlpha(alpha);
  }

  void SpeedDisplay::updateBGAlpha() {
    bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
  }



}


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::SpeedDisplay,rviz::Display )
// END_TUTORIAL
