//
// Created by matt on 10/17/16.
//

#include <cmath>
#include <QtGui/QPainter>
#include "CompassWidget.h"

namespace rr_rviz_plugins {

  CompassWidget::CompassWidget(QWidget *parent)
    : QWidget(parent),
      heading(M_PI / 2)
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
    painter.setPen(Qt::black);

    // draw background
    painter.drawRect(QRect{-1, -1, w+2, h+2});

    int circle_radius = ( std::min(w,h) / 2 ) - 2;

    QPoint circle_center{w/2,h/2};
    painter.drawEllipse(circle_center,circle_radius, circle_radius);

    QPoint needle_end{static_cast<int>(circle_radius * cos(heading)), static_cast<int>(-1 * circle_radius * sin(heading))};
    needle_end += circle_center;

    painter.drawLine(circle_center, needle_end);

    painter.setPen(Qt::gray);
    painter.drawText(circle_center+QPointF{-5, 15.0-circle_radius}, "N");
    painter.drawText(circle_center+QPointF{-5, circle_radius-15.0}, "S");
    painter.drawText(circle_center+QPointF{circle_radius-15.0,  5}, "E");
    painter.drawText(circle_center+QPointF{15.0-circle_radius,  5}, "W");
  }

  void CompassWidget::setHeading(float heading)
  {
    this->heading = heading + ( M_PI / 2 );
    update();
  }

}
