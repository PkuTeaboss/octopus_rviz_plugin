// Cheng Zhang

#include "overlay_display.h"
#include <rviz/uniform_string_stream.h>

namespace octopus_rviz_plugin
{

	OverlayDisplay::OverlayDisplay()
	: rviz::Display(), update_required_(false)
	{

	}

	OverlayDisplay::~OverlayDisplay() {

	}

	void OverlayDisplay::initProperties() {
		size_property_ = new rviz::IntProperty(
			"height", 100, "", this, SLOT(updateSize())
			);
		left_property_ = new rviz::IntProperty(
			"left", 10, "", this, SLOT(updateLeft())
			);
		top_property_ = new rviz::IntProperty(
			"top", 10, "", this, SLOT(updateTop())
			);

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

	void OverlayDisplay::onInitialize() {

		static int count = 0;
		rviz::UniformStringStream ss;
		ss << "OverlayDisplay" << count++;
		overlay_.reset(new OverlayObject(ss.str()));

		onEnable();
		updateSize();
		updateLeft();
		updateTop();
		updateFGColor();
		updateFGAlpha();
		updateBGColor();
		updateBGAlpha();

		overlay_->updateTextureSize(width_, height_);
		overlay_->setPosition(left_, top_);
		overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
	}

	void OverlayDisplay::update(float wall_dt, float ros_dt) {
		if (update_required_) {
			update_required_ = false;
			overlay_->updateTextureSize(width_, height_);
			overlay_->setPosition(left_, top_);
			overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
			draw();
		}
	}

	void OverlayDisplay::onEnable() {
		overlay_->show();
	}

	void OverlayDisplay::onDisable() {
		overlay_->hide();
	}

	void OverlayDisplay::updateSize() {
		int size = size_property_->getInt();
		if (size <= 32 || size >= 1024) {
			return;
		}
		width_ = size_property_->getInt();
		height_ = size_property_->getInt();
		update_required_ = true;
	}

	void OverlayDisplay::updateLeft() {
		left_ = left_property_->getInt();
		update_required_ = true;
	}

	void OverlayDisplay::updateTop() {
		top_ = top_property_->getInt();
		update_required_ = true;
	}

	void OverlayDisplay::updateFGColor() {
		int alpha = fg_color_.alpha();
		fg_color_ = fg_color_property_->getColor();
		fg_color_.setAlpha(alpha);
		update_required_ = true;
	}

	void OverlayDisplay::updateFGAlpha() {
		fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
		update_required_ = true;
	}

	void OverlayDisplay::updateBGColor() {
		int alpha = bg_color_.alpha();
		bg_color_ = bg_color_property_->getColor();
		bg_color_.setAlpha(alpha);
		update_required_ = true;
	}

	void OverlayDisplay::updateBGAlpha() {
		bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
		update_required_ = true;
	}


	bool OverlayDisplay::isInRegion(int x, int y)
	{
		return (top_ < y && top_ + height_ > y &&
			left_ < x && left_ + width_ > x);
	}

	void OverlayDisplay::movePosition(int x, int y)
	{
		top_ = y;
		left_ = x;
		update_required_ = true;
	}

	void OverlayDisplay::setPosition(int x, int y)
	{
		top_property_->setValue(y);
		left_property_->setValue(x);
		update_required_ = true;
	}

}

