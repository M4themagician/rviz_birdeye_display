//
// Created by jonas on 6/30/21 at https://github.com/teamspatzenhirn/rviz_birdeye_display.
// Forked by Lennart Evers on 10/30/2023
//

#ifndef VIZ_BIRDEYEDISPLAY_HPP
#define VIZ_BIRDEYEDISPLAY_HPP

#include <OgreHardwarePixelBuffer.h>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "drives_image_processing_msgs/msg/map_meta_data.hpp"

#include "rviz_birdeye_display/visibility_control.hpp"

namespace rviz_birdeye_display::displays
{

  class rviz_birdeye_display_PUBLIC BirdeyeDisplay : public rviz_common::_RosTopicDisplay
  {
    Q_OBJECT

    using ImageMsg = sensor_msgs::msg::Image;
    using ParamMsg = drives_image_processing_msgs::msg::MapMetaData;

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

  private:
    Ogre::ManualObject *m_imageObject = nullptr;
    Ogre::TexturePtr m_texture;
    Ogre::MaterialPtr m_material;

    rclcpp::Subscription<ImageMsg>::SharedPtr m_imageSub;
    rclcpp::Subscription<ParamMsg>::SharedPtr m_paramSub;
    std::optional<ParamMsg> m_currentBirdeyeParam;

    int m_messagesReceived = 0;

    std::string m_materialName;
    std::string m_textureName;

    int m_currentHeight = 0;
    int m_currentWidth = 0;
  };

} // namespace rviz_birdeye_display::displays

#endif // VIZ_BIRDEYEDISPLAY_HPP
