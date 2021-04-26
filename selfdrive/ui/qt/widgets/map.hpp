#pragma once

#include <QMapboxGL>

#include <QOpenGLWidget>
#include <QPropertyAnimation>
#include <QScopedPointer>
#include <QtGlobal>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>

#include "messaging.hpp"

class MapWindow : public QOpenGLWidget
{
    Q_OBJECT

public:
  MapWindow(const QMapboxGLSettings &);
  ~MapWindow();


private:
  void initializeGL() final;
  void paintGL() final;

  QMapboxGLSettings m_settings;
  QScopedPointer<QMapboxGL> m_map;
  void mousePressEvent(QMouseEvent *ev) final;
  void mouseMoveEvent(QMouseEvent *ev) final;
  void wheelEvent(QWheelEvent *ev) final;


  bool m_sourceAdded = false;
  SubMaster *sm;
  QTimer* timer;

  // Panning
  QPointF m_lastPos;
  int pan_counter = 0;
  int zoom_counter = 0;

private slots:
  void timerUpdate();
};
