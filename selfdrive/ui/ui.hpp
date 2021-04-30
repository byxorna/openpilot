#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <sstream>

#include "nanovg.h"

#include "camerad/cameras/camera_common.h"
#include "common/mat.h"
#include "common/visionimg.h"
#include "common/modeldata.h"
#include "common/params.h"
#include "common/glutil.h"
#include "common/util.h"
#include "common/transformations/orientation.hpp"
#include "messaging.hpp"
#include "visionipc.h"
#include "visionipc_client.h"

#include "qt/sound.hpp"

#include <QObject>
#include <QTimer>

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
//define COLOR_GREEN nvgRGBA(0, 255, 0, 255)
//define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 255, 0, x)
//define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
//define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
//define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
//define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
//define COLOR_RED nvgRGBA(201, 34, 49, 255)

// https://coolors.co/26547c-ef476f-ffd166-06d6a0-fcfcfc
#define COLOR_BLUE nvgRGBA(38, 84, 124, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(38, 84, 124, x)
#define COLOR_RED nvgRGBA(239, 71, 111, 255)
#define COLOR_RED_ALPHA(x) nvgRGBA(239, 71, 111, x)
#define COLOR_YELLOW nvgRGBA(255, 209, 102, 255)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(255, 209, 102, x)
#define COLOR_GREEN nvgRGBA(6, 214, 160, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(6, 214, 160, x)
#define COLOR_WHITE nvgRGBA(252, 252, 252, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(252, 252, 252, x)

typedef struct Rect {
  int x, y, w, h;
  int centerX() const { return x + w / 2; }
  int centerY() const { return y + h / 2; }
  int right() const { return x + w; }
  int bottom() const { return y + h; }
  bool ptInRect(int px, int py) const {
    return px >= x && px < (x + w) && py >= y && py < (y + h);
  }
} Rect;

const int bdr_s = 30;
const int header_h = 420;
const int footer_h = 280;

const int UI_FREQ = 20;   // Hz

typedef enum NetStatus {
  NET_CONNECTED,
  NET_DISCONNECTED,
  NET_ERROR,
} NetStatus;

typedef enum UIStatus {
  STATUS_OFFROAD,
  STATUS_DISENGAGED,
  STATUS_ENGAGED,
  STATUS_WARNING,
  STATUS_ALERT,
} UIStatus;

static std::map<UIStatus, NVGcolor> bg_colors = {
  {STATUS_OFFROAD, nvgRGBA(0x0, 0x0, 0x0, 0xff)},
  {STATUS_DISENGAGED, nvgRGBA(0x17, 0x33, 0x49, 0xc8)},
  {STATUS_ENGAGED, nvgRGBA(0x17, 0x86, 0x44, 0xf1)},
  {STATUS_WARNING, nvgRGBA(0xDA, 0x6F, 0x25, 0xf1)},
  {STATUS_ALERT, nvgRGBA(0xC9, 0x22, 0x31, 0xf1)},
};

typedef struct {
  float x, y;
} vertex_data;

typedef struct {
  vertex_data v[TRAJECTORY_SIZE * 2];
  int cnt;
} line_vertices_data;

typedef struct UIScene {

  mat3 view_from_calib;
  bool world_objects_visible;

  bool is_rhd;
  bool driver_view;

  // @byxorna start
  int lead_status;
  float lead_d_rel, lead_v_rel;
  float angleSteers;
  bool brakeLights;
  float angleSteersDes;
  bool recording;
  float gpsAccuracyUblox;
  float altitudeUblox;
  int engineRPM;
  bool steerOverride;
  float output_scale;
  float steeringTorqueEps;
  float aEgo;
  float cpuTemp;
  int cpuPerc;
  // @byxorna end

  std::string alert_text1;
  std::string alert_text2;
  std::string alert_type;
  float alert_blinking_rate;
  cereal::ControlsState::AlertSize alert_size;

  cereal::PandaState::PandaType pandaType;
  NetStatus athenaStatus;

  cereal::DeviceState::Reader deviceState;
  cereal::RadarState::LeadData::Reader lead_data[2];
  cereal::CarState::Reader car_state;
  cereal::ControlsState::Reader controls_state;
  cereal::DriverState::Reader driver_state;
  cereal::DriverMonitoringState::Reader dmonitoring_state;

  // gps
  int satelliteCount;
  bool gpsOK;

  // modelV2
  float lane_line_probs[4];
  float road_edge_stds[2];
  line_vertices_data track_vertices;
  line_vertices_data lane_line_vertices[4];
  line_vertices_data road_edge_vertices[2];

  // lead
  vertex_data lead_vertices[2];

  float light_sensor, accel_sensor, gyro_sensor;
  bool started, ignition, is_metric, longitudinal_control, end_to_end;
  uint64_t started_frame;
} UIScene;

typedef struct UIState {
  VisionIpcClient * vipc_client;
  VisionIpcClient * vipc_client_front;
  VisionIpcClient * vipc_client_rear;
  VisionBuf * last_frame;

  // framebuffer
  int fb_w, fb_h;

  // NVG
  NVGcontext *vg;

  // images
  std::map<std::string, int> images;

  std::unique_ptr<SubMaster> sm;

  std::unique_ptr<Sound> sound;
  UIStatus status;
  UIScene scene;

  // graphics
  std::unique_ptr<GLShader> gl_shader;
  std::unique_ptr<EGLImageTexture> texture[UI_BUF_COUNT];

  GLuint frame_vao[2], frame_vbo[2], frame_ibo[2];
  mat4 rear_frame_mat, front_frame_mat;

  bool awake;

  Rect video_rect, viz_rect;
  float car_space_transform[6];
  bool wide_camera;
  float zoom;
} UIState;


class QUIState : public QObject {
  Q_OBJECT

public:
  QUIState(QObject* parent = 0);

  // TODO: get rid of this, only use signal
  inline static UIState ui_state = {0};

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = true;
};


// device management class

class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);

private:
  // auto brightness
  const float accel_samples = 5*UI_FREQ;

  bool awake;
  int awake_timeout = 0;
  float accel_prev = 0;
  float gyro_prev = 0;
  float brightness_b = 0;
  float brightness_m = 0;
  float last_brightness = 0;
  FirstOrderFilter brightness_filter;

  QTimer *timer;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);

signals:
  void displayPowerChanged(bool on);

public slots:
  void setAwake(bool on, bool reset);
  void update(const UIState &s);
};
