//
// Created by matt on 10/17/16.
//

#ifndef PROJECT_COMPASS_PANEL_H
#define PROJECT_COMPASS_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/MagneticField.h>
#include "CompassWidget.h"

namespace rr_rviz_plugins {

  class CompassPanel : public rviz::Panel {

      Q_OBJECT

  public:
      CompassPanel(QWidget *parent = nullptr);

      virtual void load(const rviz::Config &config) override;

      virtual void save(rviz::Config config) const override;

  protected:

      void subscribeToTopic(const std::string &topic);

      void magneticFieldCallback(const sensor_msgs::MagneticFieldConstPtr msg);

      CompassWidget *compass_widget;

      ros::Subscriber mag_subscriber;

      ros::NodeHandle nh;

  };

}


#endif //PROJECT_COMPASS_PANEL_H
