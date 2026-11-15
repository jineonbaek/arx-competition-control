#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgreRenderQueue.h>
#include <OgreTechnique.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_default_plugins/displays/polygon/polygon_display.hpp>

namespace arx_waypoints {
class ForegroundPolygon : public rviz_default_plugins::displays::PolygonDisplay {
public:
  ForegroundPolygon() {
    priority_ = new rviz_common::properties::IntProperty(
      "Draw Priority", 1, "Higher values draw later within the foreground queue.", this);
    priority_->setMin(0);
    priority_->setMax(65535);
  }

protected:
  void processMessage(geometry_msgs::msg::PolygonStamped::ConstSharedPtr message) override {
    PolygonDisplay::processMessage(message);
    // The standard display supplies the frame transform, polygon geometry and color.
    // Only its private line material and render queue change.
    auto * pass = material_->getTechnique(0)->getPass(0);
    pass->setDepthCheckEnabled(false);
    pass->setDepthWriteEnabled(false);
    pass->setLineWidth(3.0f);
    manual_object_->setRenderQueueGroupAndPriority(
      Ogre::RENDER_QUEUE_OVERLAY, static_cast<unsigned short>(priority_->getInt()));
  }

private:
  rviz_common::properties::IntProperty * priority_;
};
}  // namespace arx_waypoints

PLUGINLIB_EXPORT_CLASS(arx_waypoints::ForegroundPolygon, rviz_common::Display)
