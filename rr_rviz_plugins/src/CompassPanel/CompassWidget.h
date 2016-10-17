//
// Created by matt on 10/17/16.
//

#ifndef PROJECT_COMPASSWIDGET_H
#define PROJECT_COMPASSWIDGET_H

#include <QWidget>

namespace rr_rviz_plugins {
  class CompassWidget : public QWidget {

  Q_OBJECT
  public:
    CompassWidget(QWidget* parent = nullptr);

    virtual void paintEvent( QPaintEvent* event ) override;

  public Q_SLOTS:
    void setHeading(float heading);

  protected:
    float heading;

  };
}


#endif //PROJECT_COMPASSWIDGET_H
