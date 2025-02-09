#pragma once

#include <map>
#include <QSoundEffect>
#include "cereal/gen/cpp/log.capnp.h"

typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

class Sound {
public:
  Sound();
  void play(AudibleAlert alert);
  void stop();
  float volume = 0;

private:
  std::map<AudibleAlert, std::pair<QString, int>> sound_map {
    // AudibleAlert, (file path, loop count)
    {AudibleAlert::CHIME_DISENGAGE, {"../assets/sounds/disengaged.wav", 0}},
    {AudibleAlert::CHIME_ENGAGE, {"../assets/sounds/engaged.wav", 0}},
    {AudibleAlert::CHIME_WARNING1, {"../assets/sounds/warning_1.wav", 0}},
    {AudibleAlert::CHIME_WARNING2, {"../assets/sounds/warning_2.wav", 0}},
    {AudibleAlert::CHIME_WARNING2_REPEAT, {"../assets/sounds/warning_2.wav", -1}},
    {AudibleAlert::CHIME_WARNING_REPEAT, {"../assets/sounds/warning_repeat.wav", -1}},
    {AudibleAlert::CHIME_ERROR, {"../assets/sounds/error.wav", 0}},
    {AudibleAlert::CHIME_PROMPT, {"../assets/sounds/error.wav", 0}}
  };

  std::map<AudibleAlert, QSoundEffect> sounds;
};
