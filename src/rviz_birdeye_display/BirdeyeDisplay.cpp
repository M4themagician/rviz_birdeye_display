//
// Created by jonas on 6/30/21 at https://github.com/teamspatzenhirn/rviz_birdeye_display.
// Forked by Lennart Evers on 10/30/2023
//

#include "rviz_birdeye_display/BirdeyeDisplay.hpp"

#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreMesh.h>
#include <OgreSceneNode.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <opencv2/opencv.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <sensor_msgs/image_encodings.hpp>

constexpr auto RESOURCEGROUP_NAME = "rviz_rendering";

namespace rviz_birdeye_display::displays
{

    int cvTypeFromEncoding(const std::string &encoding)
    {
        using namespace sensor_msgs::image_encodings;
        if (encoding.rfind("32FC", 0) == 0)
        {
            return CV_32FC(numChannels(encoding));
        }
        if (bitDepth(encoding) == 8)
        {
            // This case covers all bgr/rgb/bayer 8bit cases. OpenCV does not differentiate between them.
            return CV_8UC(numChannels(encoding));
        }

        throw std::invalid_argument{"OpenCV Type for encoding could not be found"};
    }

    std::string parentTopic(const std::string &topic)
    {
        return topic.substr(0, topic.find_last_of('/'));
    }

    BirdeyeDisplay::BirdeyeDisplay()
    {
        std::string message_type = rosidl_generator_traits::name<ImageMsg>();
        topic_property_->setMessageType(QString::fromStdString(message_type));
        topic_property_->setDescription(QString::fromStdString(message_type) + " topic to subscribe to.");

        static int birdeye_count = 0;
        birdeye_count++;
        m_materialName = "BirdeyeMaterial" + std::to_string(birdeye_count);
        m_textureName = "BirdeyeTexture" + std::to_string(birdeye_count);

        m_currentBirdeyeParam = drives_image_processing_msgs::msg::MapMetaData();
        m_currentBirdeyeParam.width = 800;
        m_currentBirdeyeParam.height = 800;
        m_currentBirdeyeParam.resolution = 0.1;
        auto offset = geometry_msgs::msg::Pose2D();
        offset.x = -40;
        offset.y = 40;
        m_currentBirdeyeParam.offset = std::move(offset);
    }

    BirdeyeDisplay::~BirdeyeDisplay()
    {
        if (initialized())
        {
            scene_manager_->destroyManualObject(m_imageObject);
        }
        unsubscribe();
    }

    void BirdeyeDisplay::createTextures()
    {
        // assert(m_currentBirdeyeParam.has_value());

        if (m_currentBirdeyeParam.height == m_currentHeight and m_currentBirdeyeParam.width == m_currentWidth)
        {
            return;
        }

        m_texture = Ogre::TextureManager::getSingleton().createManual(
            m_textureName, RESOURCEGROUP_NAME, Ogre::TEX_TYPE_2D, m_currentBirdeyeParam.width,
            m_currentBirdeyeParam.height, 1, 0, Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        m_material = rviz_rendering::MaterialManager::createMaterialWithNoLighting(m_materialName);

        auto rpass = m_material->getTechniques()[0]->getPasses()[0];
        rpass->createTextureUnitState(m_textureName);
        rpass->setCullingMode(Ogre::CULL_NONE);
        rpass->setEmissive(Ogre::ColourValue::White);
        rpass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        m_currentHeight = m_currentBirdeyeParam.height;
        m_currentWidth = m_currentBirdeyeParam.width;
    }

    void BirdeyeDisplay::onInitialize()
    {
        _RosTopicDisplay::onInitialize();

        m_imageObject = scene_manager_->createManualObject();
        m_imageObject->setDynamic(true);
        scene_node_->attachObject(m_imageObject);
    }

    void BirdeyeDisplay::reset()
    {
        _RosTopicDisplay::reset();
        m_imageObject->clear();
        m_messagesReceived = 0;
    }

    void BirdeyeDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {

        // if (!m_currentBirdeyeParam.has_value())
        // {
        //     setStatus(rviz_common::properties::StatusProperty::Error, "Params", QString("No BirdEyeParam"));
        //     return;
        // }
        setStatus(rviz_common::properties::StatusProperty::Ok, "Params", QString("OK"));

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform("ego_vehicle", msg->header.stamp,
                                                       position, orientation))
        {
            setMissingTransformToFixedFrame("ego_vehicle");
            return;
        }
        setTransformOk();

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        createTextures();

        m_imageObject->clear();
        m_imageObject->estimateVertexCount(4);
        m_imageObject->begin(m_material->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, "rviz_rendering");

        auto xOffset = m_currentBirdeyeParam.offset.x; // / m_currentBirdeyeParam->resolution;
        auto yOffset = m_currentBirdeyeParam.offset.y; // / m_currentBirdeyeParam->resolution;
        auto height = m_currentBirdeyeParam.height * m_currentBirdeyeParam.resolution;
        auto width = m_currentBirdeyeParam.width * m_currentBirdeyeParam.resolution;

        /**
         *        birdeye-height
         *     2------------------1
         *     |                  |
         *     |                  | birdeye-width
         *     |                  |
         *     3------------------0
         *     |---------|     I
         *      y-offset ^     I birdeye-x-offset
         *               |     I
         *   driving dir |     I
         *               |     I
         *            Vehicle  I
         */

        // 0
        m_imageObject->position(xOffset, height - yOffset, 0);
        m_imageObject->textureCoord(0, 0);

        // 1
        m_imageObject->position(xOffset + width, height - yOffset, 0);
        m_imageObject->textureCoord(1, 0);

        // 2
        m_imageObject->position(xOffset + width, -yOffset, 0);
        m_imageObject->textureCoord(1, 1);

        // 3
        m_imageObject->position(xOffset, -yOffset, 0);
        m_imageObject->textureCoord(0, 1);
        m_imageObject->end();

        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = m_texture->getBuffer();

        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

        cv::Mat input(msg->height, msg->width, cvTypeFromEncoding(msg->encoding), (void *)msg->data.data(), msg->step);

        cv::Mat textureMat(m_currentHeight, m_currentWidth, CV_8UC4, (void *)pixelBox.data);

        if (sensor_msgs::image_encodings::numChannels(msg->encoding) == 1)
        {
            cv::cvtColor(input, textureMat, cv::COLOR_GRAY2BGRA, 4);
        }
        else if (msg->encoding.rfind("rgb", 0) == 0)
        {
            cv::cvtColor(input, textureMat, cv::COLOR_RGB2BGRA, 4);
        }
        else if (msg->encoding.rfind("bgr", 0) == 0)
        {
            cv::cvtColor(input, textureMat, cv::COLOR_BGR2BGRA, 4);
        }
        else
        {
            throw std::runtime_error{"Unknown encoding: " + msg->encoding};
        }

        // Unlock the pixel buffer
        pixelBuffer->unlock();
    }

    void BirdeyeDisplay::subscribe()
    {
        if (!isEnabled())
        {
            return;
        }

        if (topic_property_->isEmpty())
        {
            setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: Empty topic name"));
            return;
        }

        try
        {
            rclcpp::SensorDataQoS qos;
            m_imageSub = rviz_ros_node_.lock()->get_raw_node()->create_subscription<ImageMsg>(
                topic_property_->getTopicStd(), qos,
                [this](ImageMsg::ConstSharedPtr msg)
                { incomingMessage(msg); });

            // m_paramSub = rviz_ros_node_.lock()->get_raw_node()->create_subscription<ParamMsg>(
            //     parentTopic(topic_property_->getTopicStd()) + "/map_metadata", qos,
            //     [this](ParamMsg ::ConstSharedPtr msg)
            //     { this->m_currentBirdeyeParam = *msg; });

            setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
        }
        catch (std::exception &e)
        {
            setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: ") + e.what());
        }
    }

    void BirdeyeDisplay::unsubscribe()
    {
        m_imageSub.reset();
        m_paramSub.reset();
    }

    void BirdeyeDisplay::updateTopic()
    {
        resetSubscription();
    }

    void BirdeyeDisplay::resetSubscription()
    {
        unsubscribe();
        reset();
        subscribe();
        context_->queueRender();
    }

    void BirdeyeDisplay::incomingMessage(const ImageMsg::ConstSharedPtr &msg)
    {
        if (!msg)
        {
            return;
        }

        ++m_messagesReceived;
        setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
                  QString::number(m_messagesReceived) + " messages received");

        processMessage(msg);
    }

    void BirdeyeDisplay::setTopic(const QString &topic, const QString & /* datatype */)
    {
        topic_property_->setString(topic);
    }

    void BirdeyeDisplay::onEnable()
    {
        subscribe();
    }

    void BirdeyeDisplay::onDisable()
    {
        unsubscribe();
        reset();
    }

} // namespace rviz_birdeye_display::displays

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_birdeye_display::displays::BirdeyeDisplay, rviz_common::Display)
