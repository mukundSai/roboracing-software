//
// Created by matt on 10/17/16.
//

#include <QtGui/QPainter>
#include "CompassWidget.h"

namespace rr_rviz_plugins {

  CompassWidget::CompassWidget(QWidget *parent)
    : QWidget(parent),
      heading(0)
  {
  }

  void CompassWidget::paintEvent(QPaintEvent *event)
  {
    QColor background;
    if(isEnabled())
    {
      background = Qt::white;
    } else {
      background = Qt::lightGray;
    }

    int w = width();
    int h = height();

    QPainter painter(this);
    painter.setBrush(background);

    // draw background
    painter.drawRect(QRect(0, 0, w, h));

    // TODO draw compass
  }

  void CompassWidget::setHeading(float heading)
  {
    this->heading = heading;
  }

}