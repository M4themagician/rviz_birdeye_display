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

        m_properties.alpha = std::make_unique<rviz_common::properties::FloatProperty>("Alpha", 1.0f, "Opacity of Map", this, SLOT(updateAlpha()));
        m_properties.alpha->setMin(0.0f);
        m_properties.alpha->setMax(1.0f);
        m_properties.z = std::make_unique<rviz_common::properties::FloatProperty>("Z", 0.0f, "Z Height of Texture", this, SLOT(updateZ()));
        m_properties.z->setMin(-1.0f);
        m_properties.z->setMax(1.0f);

        m_properties.raised_index = std::make_unique<rviz_common::properties::IntProperty>("Raised Index", 255, "Index from which display will be raised.", this, SLOT(updateRaisedIndex()));
        m_properties.raised_index->setMin(0);
        m_properties.raised_index->setMax(255);

        m_properties.raised_height = std::make_unique<rviz_common::properties::FloatProperty>("Raised Height", 0.2f, "Added Z Height of raised Texture", this, SLOT(updateRaisedHeight()));
        m_properties.raised_height->setMin(0.0f);
        m_properties.raised_height->setMax(10.0f);

        m_properties.colormap = std::make_unique<rviz_common::properties::EditableEnumProperty>("Colormap", "Parula", "The Colormap to use to visualize the Category Grid Map.", this, SLOT(updateColormap()));
        for (int cmap_ = 0; cmap_ <= 21; cmap_++)
        {
            std::string cmap_name;
            switch (cmap_)
            {
            case 0:
                cmap_name = "Autumn";
                break;
            case 1:
                cmap_name = "Bone";
                break;
            case 2:
                cmap_name = "Jet";
                break;
            case 3:
                cmap_name = "Winter";
                break;
            case 4:
                cmap_name = "Rainbow";
                break;
            case 5:
                cmap_name = "Ocean";
                break;
            case 6:
                cmap_name = "Summer";
                break;
            case 7:
                cmap_name = "Spring";
                break;
            case 8:
                cmap_name = "Cool";
                break;
            case 9:
                cmap_name = "HSV";
                break;
            case 10:
                cmap_name = "Pink";
                break;
            case 11:
                cmap_name = "Hot";
                break;
            case 12:
                cmap_name = "Parula";
                break;
            case 13:
                cmap_name = "Magma";
                break;
            case 14:
                cmap_name = "Inferno";
                break;
            case 15:
                cmap_name = "Plasma";
                break;
            case 16:
                cmap_name = "Viridis";
                break;
            case 17:
                cmap_name = "Cividis";
                break;
            case 18:
                cmap_name = "Twilight";
                break;
            case 19:
                cmap_name = "Twilight shifted";
                break;
            case 20:
                cmap_name = "Turbo";
                break;
            case 21:
                cmap_name = "Deepgreen";
                break;
            default:
                break;
            }
            m_properties.colormap.get()->addOption(cmap_name.c_str());
        }

        updateColormap();
        updateAlpha();
        updateRaisedHeight();
        updateRaisedIndex();

        std::string message_type = rosidl_generator_traits::name<ImageMsg>();
        topic_property_->setMessageType(QString::fromStdString(message_type));
        topic_property_->setDescription(QString::fromStdString(message_type) + " topic to subscribe to.");

        static int birdeye_count{0};
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

    void BirdeyeDisplay::updateAlpha()
    {
        m_alpha = m_properties.alpha->getFloat();
    }

    void BirdeyeDisplay::updateZ()
    {
        m_z = m_properties.z->getFloat();
    }

    void BirdeyeDisplay::updateRaisedIndex()
    {
        m_raised_index = m_properties.raised_index->getInt();
    }

    void BirdeyeDisplay::updateRaisedHeight()
    {
        m_raised_height = m_properties.raised_height->getFloat();
    }

    void
    BirdeyeDisplay::updateColormap()
    {
        auto cmap_str = m_properties.colormap->getString();
        int cmap_int;
        if (cmap_str == "Autumn")
        {
            cmap_int = 0;
        }
        else if (cmap_str == "Bone")
        {
            cmap_int = 1;
        }
        else if (cmap_str == "Jet")
        {
            cmap_int = 2;
        }
        else if (cmap_str == "Winter")
        {
            cmap_int = 3;
        }
        else if (cmap_str == "Rainbow")
        {
            cmap_int = 4;
        }
        else if (cmap_str == "Ocean")
        {
            cmap_int = 5;
        }
        else if (cmap_str == "Summer")
        {
            cmap_int = 6;
        }
        else if (cmap_str == "Spring")
        {
            cmap_int = 7;
        }
        else if (cmap_str == "Cool")
        {
            cmap_int = 8;
        }
        else if (cmap_str == "HSV")
        {
            cmap_int = 9;
        }
        else if (cmap_str == "Pink")
        {
            cmap_int = 10;
        }
        else if (cmap_str == "Hot")
        {
            cmap_int = 11;
        }
        else if (cmap_str == "Parula")
        {
            cmap_int = 12;
        }
        else if (cmap_str == "Magma")
        {
            cmap_int = 13;
        }
        else if (cmap_str == "Inferno")
        {
            cmap_int = 14;
        }
        else if (cmap_str == "Plasma")
        {
            cmap_int = 15;
        }
        else if (cmap_str == "Viridis")
        {
            cmap_int = 16;
        }
        else if (cmap_str == "Cividis")
        {
            cmap_int = 17;
        }
        else if (cmap_str == "Twilight")
        {
            cmap_int = 18;
        }
        else if (cmap_str == "Twilight shifted")
        {
            cmap_int = 19;
        }
        else if (cmap_str == "Turbo")
        {
            cmap_int = 20;
        }
        else
        {
            cmap_int = 21;
        }

        m_colormap = cmap_int;
    }

    void BirdeyeDisplay::createTextures()
    {
        // assert(m_currentBirdeyeParam.has_value());

        if (m_currentBirdeyeParam.height == m_currentHeight and m_currentBirdeyeParam.width == m_currentWidth and m_currentBirdeyeParam.resolution == m_currentResolution)
        {
            return;
        }
        static int birdeye_count{0};
        birdeye_count++;
        m_materialName = "BirdeyeMaterial" + std::to_string(birdeye_count);
        m_textureName = "BirdeyeTexture" + std::to_string(birdeye_count);

        m_texture = Ogre::TextureManager::getSingleton().createManual(
            m_textureName, RESOURCEGROUP_NAME, Ogre::TEX_TYPE_2D, m_currentBirdeyeParam.width,
            m_currentBirdeyeParam.height, 1, 0, Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        m_material = rviz_rendering::MaterialManager::createMaterialWithNoLighting(m_materialName);

        auto rpass = m_material->getTechniques()[0]->getPasses()[0];
        rpass->createTextureUnitState(m_textureName);
        rpass->setCullingMode(Ogre::CULL_NONE);
        rpass->setEmissive(Ogre::ColourValue::Black);
        rpass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        m_texture_raised = Ogre::TextureManager::getSingleton().createManual(
            m_textureName + "_raised", RESOURCEGROUP_NAME, Ogre::TEX_TYPE_2D, m_currentBirdeyeParam.width,
            m_currentBirdeyeParam.height, 1, 0, Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
        m_material_raised = rviz_rendering::MaterialManager::createMaterialWithNoLighting(m_materialName + "_raised");
        auto rpass_raised = m_material_raised->getTechniques()[0]->getPasses()[0];
        rpass_raised->createTextureUnitState(m_textureName + "_raised");
        rpass_raised->setCullingMode(Ogre::CULL_NONE);
        rpass_raised->setEmissive(Ogre::ColourValue::Black);
        rpass_raised->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        m_currentHeight = m_currentBirdeyeParam.height;
        m_currentWidth = m_currentBirdeyeParam.width;
        m_currentResolution = m_currentBirdeyeParam.resolution;
    }

    void BirdeyeDisplay::onInitialize()
    {
        _RosTopicDisplay::onInitialize();

        m_imageObject = scene_manager_->createManualObject();
        m_imageObject->setDynamic(true);
        scene_node_->attachObject(m_imageObject);

        m_imageObject_raised = scene_manager_->createManualObject();
        m_imageObject_raised->setDynamic(true);
        scene_node_->attachObject(m_imageObject_raised);
    }

    void BirdeyeDisplay::reset()
    {
        _RosTopicDisplay::reset();
        m_imageObject->clear();
        m_imageObject_raised->clear();
        m_messagesReceived = 0;
    }

    void BirdeyeDisplay::processMessage(const drives_image_processing_msgs::msg::CategoryGridMap::ConstSharedPtr &msg)
    {

        // if (!m_currentBirdeyeParam.has_value())
        // {
        //     setStatus(rviz_common::properties::StatusProperty::Error, "Params", QString("No BirdEyeParam"));
        //     return;
        // }

        m_currentBirdeyeParam = msg->metadata;
        m_num_classes = m_currentBirdeyeParam.categories.size();
        setStatus(rviz_common::properties::StatusProperty::Ok, "Params", QString("OK"));

        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp,
                                                       position, orientation))
        {
            setMissingTransformToFixedFrame(msg->header.frame_id);
            return;
        }
        setTransformOk();

        scene_node_->setPosition(position);
        scene_node_->setOrientation(orientation);

        createTextures();

        m_imageObject->clear();
        m_imageObject->estimateVertexCount(4);
        m_imageObject->begin(m_material->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, "rviz_rendering");

        auto height = m_currentBirdeyeParam.height * m_currentBirdeyeParam.resolution;
        auto width = m_currentBirdeyeParam.width * m_currentBirdeyeParam.resolution;
        auto xOffset = m_currentBirdeyeParam.offset.x; // / m_currentBirdeyeParam->resolution;
        auto yOffset = m_currentBirdeyeParam.offset.y; // / m_currentBirdeyeParam->resolution;

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
        m_imageObject->position(xOffset, height + yOffset, m_z);
        m_imageObject->textureCoord(0, 0);

        // 1
        m_imageObject->position(xOffset + width, height + yOffset, m_z);
        m_imageObject->textureCoord(1, 0);

        // 2
        m_imageObject->position(xOffset + width, +yOffset, m_z);
        m_imageObject->textureCoord(1, 1);

        // 3
        m_imageObject->position(xOffset, +yOffset, m_z);
        m_imageObject->textureCoord(0, 1);
        m_imageObject->end();

        m_imageObject_raised->clear();
        m_imageObject_raised->estimateVertexCount(4);
        m_imageObject_raised->begin(m_material_raised->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN, "rviz_rendering");
        m_imageObject_raised->position(xOffset, height + yOffset, m_z + m_raised_height);
        m_imageObject_raised->textureCoord(0, 0);
        m_imageObject_raised->position(xOffset + width, height + yOffset, m_z + m_raised_height);
        m_imageObject_raised->textureCoord(1, 0);
        m_imageObject_raised->position(xOffset + width, +yOffset, m_z + m_raised_height);
        m_imageObject_raised->textureCoord(1, 1);
        m_imageObject_raised->position(xOffset, +yOffset, m_z + m_raised_height);
        m_imageObject_raised->textureCoord(0, 1);
        m_imageObject_raised->end();

        Ogre::HardwarePixelBufferSharedPtr pixelBuffer = m_texture->getBuffer();
        Ogre::HardwarePixelBufferSharedPtr pixelBuffer_raised = m_texture_raised->getBuffer();

        // Lock the pixel buffer and get a pixel box
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        pixelBuffer_raised->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();
        const Ogre::PixelBox &pixelBox_raised = pixelBuffer_raised->getCurrentLock();

        auto img_msg = msg->map;

        cv::Mat input_categories(img_msg.height, img_msg.width, cvTypeFromEncoding(img_msg.encoding), (void *)img_msg.data.data(), img_msg.step);

        cv::Mat input_low, input_raised;

        cv::Mat low, raised;
        input_categories.copyTo(low, input_categories < m_raised_index);
        input_categories.copyTo(raised, input_categories >= m_raised_index);

        cv::applyColorMap(255 / m_num_classes * low, input_low, m_colormap);
        cv::applyColorMap(255 / m_num_classes * raised, input_raised, m_colormap);

        cv::Mat textureMat_low(m_currentHeight, m_currentWidth, CV_8UC4, (void *)pixelBox.data);
        cv::Mat textureMat_raised(m_currentHeight, m_currentWidth, CV_8UC4, (void *)pixelBox_raised.data);

        cv::cvtColor(input_low, textureMat_low, cv::COLOR_BGR2BGRA, 4);
        cv::cvtColor(input_raised, textureMat_raised, cv::COLOR_BGR2BGRA, 4);
        // Split the image for access to alpha channel
        std::vector<cv::Mat> channels_low(4);
        std::vector<cv::Mat> channels_raised(4);
        cv::split(textureMat_low, channels_low);
        cv::split(textureMat_raised, channels_raised);
        // Assign the mask to the last channel of the image
        channels_low[3].setTo(m_alpha * 255);
        channels_low[3].setTo(0, low == 0);

        channels_raised[3].setTo(m_alpha * 255);
        channels_raised[3].setTo(0, raised == 0);

        // Finally concat channels for rgba image
        cv::merge(channels_low, textureMat_low);
        cv::merge(channels_raised, textureMat_raised);

        // Unlock the pixel buffer
        pixelBuffer->unlock();
        pixelBuffer_raised->unlock();
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
                [this](drives_image_processing_msgs::msg::CategoryGridMap::ConstSharedPtr msg)
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
