// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <QApplication>
#include <QMenu>
#include <QTimer>
#include <ros/ros.h>
#include <rviz/tool_manager.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>

#include "overlay_picker_tool.h"
#include "speed_display.h"
#include "vehicle_control_display.h"
#include "speed_limit_display.h"
#include "overlay_display.h"

namespace octopus_rviz_plugin
{
  OverlayPickerTool::OverlayPickerTool()
  : is_moving_(false), shift_pressing_(false), rviz::Tool()
  {

  }

  int OverlayPickerTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
  {
    if (event->type() == QEvent::KeyPress && event->key() == 16777248) { // sift
      shift_pressing_ = !shift_pressing_;
    }
    return 0;
  }
  
  
  int OverlayPickerTool::processMouseEvent(rviz::ViewportMouseEvent& event)
  {
    if (event.left() && event.leftDown()) {
      if (!is_moving_) {
        onClicked(event);
      }
    }
    else if (event.left() && is_moving_) {
      onMove(event);
    }
    else if (is_moving_ && !(event.left() && event.leftDown())) {
      onRelease(event);
    }
    return 0;
  }

  bool OverlayPickerTool::handleDisplayClick(rviz::Property* property, rviz::ViewportMouseEvent& event)
  {
    if (isPropertyType<rviz::DisplayGroup>(property)) {
      rviz::DisplayGroup* group_property = isPropertyType<rviz::DisplayGroup>(property);
      for (int i = 0; i < group_property->numChildren(); i++) {
        if (handleDisplayClick(group_property->childAt(i), event)) {
          return true;
        }
      }
    }
    else {
      if (startMovement<SpeedDisplay>(property, event, "speed_display")) {
        return true;
      }
      if (startMovement<SpeedLimitDisplay>(property, event, "speed_limit_display")) {
        return true;
      }
      if (startMovement<VehicleControlDisplay>(property, event, "vehicle_control_display")) {
        return true;
      }
      if (startMovement<OverlayDisplay>(property, event, "overlay_display")) {
        return true;
      }
    }
    return false;
  }

  void OverlayPickerTool::onClicked(rviz::ViewportMouseEvent& event)
  {
    ROS_DEBUG("onClicked");
    is_moving_ = true;
    ROS_DEBUG("clicked: (%d, %d)", event.x, event.y);
    // check the active overlay plugin
    rviz::DisplayGroup* display_group = context_->getRootDisplayGroup();
    handleDisplayClick(display_group, event);
  }

  void OverlayPickerTool::onMove(rviz::ViewportMouseEvent& event)
  {
    ROS_DEBUG("onMove");
    ROS_DEBUG("moving: (%d, %d)", event.x, event.y);
    if (target_property_) {
      if (target_property_type_ == "speed_display") {
        movePosition<SpeedDisplay>(event);
      } 
      if (target_property_type_ == "speed_limit_display") {
        movePosition<SpeedLimitDisplay>(event);
      } 
      if (target_property_type_ == "vehicle_control_display") {
        movePosition<VehicleControlDisplay>(event);
      }
      if (target_property_type_ == "overlay_display") {
        movePosition<OverlayDisplay>(event);
      }
    }
  }
  
  void OverlayPickerTool::onRelease(rviz::ViewportMouseEvent& event)
  {
    ROS_DEBUG("onRelease");
    is_moving_ = false;
    ROS_DEBUG("released: (%d, %d)", event.x, event.y);
    if (target_property_) {
      if (target_property_type_ == "speed_display") {
        setPosition<SpeedDisplay>(event);
      }
      if (target_property_type_ == "speed_limit_display") {
        setPosition<SpeedLimitDisplay>(event);
      }
      if (target_property_type_ == "vehicle_control_display") {
        setPosition<VehicleControlDisplay>(event);
      }
      if (target_property_type_ == "overlay_display") {
        setPosition<OverlayDisplay>(event);
      }
    }
    // clear cache
    target_property_ = NULL;
    target_property_type_ = "";
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::OverlayPickerTool, rviz::Tool )
