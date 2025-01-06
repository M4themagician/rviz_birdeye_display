//
// Created by jonas on 6/30/21 at https://github.com/teamspatzenhirn/rviz_birdeye_display.
// Forked by Lennart Evers on 10/30/2023
//

#ifndef VIZ_BIRDEYEDISPLAY_HPP
#define VIZ_BIRDEYEDISPLAY_HPP

#include <opencv2/opencv.hpp>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "drives_image_processing_msgs/msg/category_grid_map.hpp"
#include <rviz_common/display.hpp>
#include <rviz_common/message_filter_display.hpp>

#include "rviz_birdeye_display/visibility_control.hpp"

namespace rviz_birdeye_display::displays
{

  class rviz_birdeye_display_PUBLIC BirdeyeDisplay : public rviz_common::_RosTopicDisplay
  {
    Q_OBJECT

    using ImageMsg = drives_image_processing_msgs::msg::CategoryGridMap;

  public:
    BirdeyeDisplay();
    ~BirdeyeDisplay() override;

  private:
    void onInitialize() override;

    void reset() override;
    void processMessage(const ImageMsg ::ConstSharedPtr &msg);

    void incomingMessage(const ImageMsg::ConstSharedPtr &msg);

    /**
     * Create new m_texture with current image size if it has changed
     */
    void createTextures();
    void subscribe();
    void unsubscribe();

    void resetSubscription();
    void updateTopic() override;
    void setTopic(const QString &topic, const QString &datatype) override;
    void onEnable() override;

    void onDisable() override;

  private Q_SLOTS:
    void updateColormap();
    void updateAlpha();
    void updateZ();

  private:
    Ogre::ManualObject *m_imageObject = nullptr;
    Ogre::TexturePtr m_texture;
    Ogre::MaterialPtr m_material;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_imageSub;
    drives_image_processing_msgs::msg::MapMetaData m_currentBirdeyeParam;

    int m_messagesReceived = 0;

    std::string m_materialName;
    std::string m_textureName;

    uint32_t m_currentHeight = 0;
    uint32_t m_currentWidth = 0;
    double m_currentResolution = 0.05;
    float m_alpha = 1.0f;
    float m_z = 0.0f;

    int m_num_classes = 11;

    int m_colormap = 0;

    struct Properties
    {
      std::unique_ptr<rviz_common::properties::EditableEnumProperty> colormap;
      std::unique_ptr<rviz_common::properties::FloatProperty> alpha;
      std::unique_ptr<rviz_common::properties::FloatProperty> z;
    } m_properties;
  };

} // namespace rviz_birdeye_display::displays

#endif // VIZ_BIRDEYEDISPLAY_HPP
