//
// Created by matt on 10/17/16.
//

#include <QtWidgets/QVBoxLayout>
#include "CompassPanel.h"

namespace rr_rviz_plugins {

  CompassPanel::CompassPanel(QWidget *parent)
      : rviz::Panel(parent)
  {
    compass_widget = new CompassWidget;

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(compass_widget);

    setLayout(layout);

    subscribeToTopic("/imu/mag");
  }

  void CompassPanel::load(const rviz::Config &config)
  {
    rviz::Panel::load(config);
  }

  void CompassPanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  void CompassPanel::magneticFieldCallback(const sensor_msgs::MagneticFieldConstPtr msg)
  {
    compass_widget->setHeading(static_cast<float>(msg->magnetic_field.z));
  }

  void CompassPanel::subscribeToTopic(const std::string &topic)
  {
    mag_subscriber = nh.subscribe(topic, 1, &CompassPanel::magneticFieldCallback, this);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rr_rviz_plugins::CompassPanel,rviz::Panel )

