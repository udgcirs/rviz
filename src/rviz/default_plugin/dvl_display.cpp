//
// Created by narcis on 17/10/18.
//

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/parse_color.h"

#include "dvl_display.h"
#include <limits>

namespace rviz
{
    DVLDisplay::DVLDisplay()
    {
        number_of_beams_ = 1;

        color_property_ = new ColorProperty( "Color", Qt::white,
                                             "Color to draw the range.",
                                             this, SLOT( updateColorAndAlpha() ));

        alpha_property_ = new FloatProperty( "Alpha", 0.5,
                                             "Amount of transparency to apply to the range.",
                                             this, SLOT( updateColorAndAlpha() ));

        buffer_length_property_ = new IntProperty( "Buffer Length", 1,
                                                   "Number of prior measurements to display.",
                                                   this, SLOT( updateBufferLength() ));
        buffer_length_property_->setMin( 1 );

//        queue_size_property_ = new IntProperty( "Queue Size", 1,
//                                                "Size of the tf message filter queue. It usually needs to be set at least as high as the number of sonar frames.",
//                                                this, SLOT( updateQueueSize() ));


    }

    void DVLDisplay::onInitialize()
    {
        MFDClass::onInitialize();
        updateBufferLength();
        updateColorAndAlpha();
    }

    DVLDisplay::~DVLDisplay()
    {
        for( size_t i = 0; i < cones_.size(); i++ )
        {
            delete cones_[ i ];
        }
    }

    void DVLDisplay::reset()
    {
        MFDClass::reset();
        updateBufferLength();
    }

//    void DVLDisplay::updateQueueSize()
//    {
//        tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
//    }

    void DVLDisplay::updateColorAndAlpha()
    {
        Ogre::ColourValue oc = color_property_->getOgreColor();
        float alpha = alpha_property_->getFloat();
        for( size_t i = 0; i < cones_.size(); i++ )
        {
            cones_[i]->setColor( oc.r, oc.g, oc.b, alpha );
        }
        context_->queueRender();
    }

    void DVLDisplay::updateBufferLength()
    {
//        int buffer_length = buffer_length_property_->getInt();
        int buffer_length = number_of_beams_;
        QColor color = color_property_->getColor();

        for( size_t i = 0; i < cones_.size(); i++ )
        {
            delete cones_[i];
        }
        cones_.resize( buffer_length );
        for( size_t i = 0; i < cones_.size(); i++ )
        {
            Shape* cone = new Shape( Shape::Cone, context_->getSceneManager(), scene_node_ );
            cones_[ i ] = cone;

            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            geometry_msgs::Pose pose;
            pose.orientation.w = 1;
            Ogre::Vector3 scale( 0, 0, 0 );
            cone->setScale( scale );
            cone->setColor( color.redF(), color.greenF(), color.blueF(), 0 );
        }
    }

    void DVLDisplay::processMessage( const sensor_msgs::DVL::ConstPtr& msg )
    {
        if (msg->beams.size() != number_of_beams_ && msg->beams.size() > 0)
        {
            number_of_beams_ = msg->beams.size();
            updateBufferLength();
        }

        for (unsigned int i = 0; i < msg->beams.size(); i++)
        {
            // Shape* cone = cones_[ messages_received_ % buffer_length_property_->getInt() ];
            Shape *cone = cones_[i];

            Ogre::Vector3 position;
            Ogre::Quaternion orientation;
            geometry_msgs::Pose pose;
            float displayed_range = 0.0;
            if (msg->beams[i].range < 0)
            {
                displayed_range = 0.0;
            }
            else
            {
                displayed_range = msg->beams[i].range;
            }

            pose.position.x = displayed_range / 2 - .008824 *
                                                    displayed_range; // .008824 fudge factor measured, must be inaccuracy of cone model.
            pose.orientation.z = 0.707;
            pose.orientation.w = 0.707;
            if (!context_->getFrameManager()->transform(msg->beams[i].frame_id, msg->header.stamp, pose, position, orientation))
            {
                ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                          msg->beams[i].frame_id.c_str(), qPrintable(fixed_frame_));
            }

            cone->setPosition(position);
            cone->setOrientation(orientation);

            double cone_width = 2.0 * displayed_range * tan(msg->beams[i].field_of_view / 2.0);
            Ogre::Vector3 scale(cone_width, displayed_range, cone_width);
            cone->setScale(scale);

            QColor color = color_property_->getColor();
            cone->setColor(color.redF(), color.greenF(), color.blueF(), alpha_property_->getFloat());
        }
    }

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::DVLDisplay,rviz::Display )
