#ifndef DVL_DISPLAY_H
#define DVL_DISPLAY_H

#include <sensor_msgs/DVL.h>

#include "rviz/message_filter_display.h"

namespace rviz
{
    class Shape;
}

namespace rviz
{

    class ColorProperty;
    class FloatProperty;
    class IntProperty;

/**
 * \class RangeDisplay
 * \brief Displays a sensor_msgs::Range message as a cone.
 */
    class DVLDisplay: public MessageFilterDisplay<sensor_msgs::DVL>
    {
        Q_OBJECT
    public:
        DVLDisplay();
        virtual ~DVLDisplay();

        /** @brief Overridden from Display. */
        virtual void reset();

    protected:
        /** @brief Overridden from Display. */
        virtual void onInitialize();

        /** @brief Overridden from MessageFilterDisplay. */
        virtual void processMessage( const sensor_msgs::DVL::ConstPtr& msg );

    private Q_SLOTS:
        void updateBufferLength();
        void updateColorAndAlpha();
        void updateQueueSize();

    private:
        std::vector<Shape* > cones_;      ///< Handles actually drawing the cones

        ColorProperty* color_property_;
        FloatProperty* alpha_property_;
        IntProperty* buffer_length_property_;
//        IntProperty* queue_size_property_;
        unsigned int number_of_beams_;
    };

} // namespace range_plugin

#endif /* DVL_DISPLAY_H */
