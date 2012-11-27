#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"
#include "ff.h"

#include "Singleton.h"

class SampleAndControl : public Singleton<SampleAndControl> {
  friend class Singleton<SampleAndControl>;
 public:
  static void chshellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void Control_(void * arg);
  void Control();
  void setEnabled(bool state);
  bool isEnabled() const { return Enabled_; }
  void setFilename(const char * name);
  char * Filename() const;

 private:
  SampleAndControl();
  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);

  Thread * Control_tp_;
  char Filename_[24];
  FIL f_;
  WORKING_AREA(waControlThread, 1024);
  bool Enabled_;
};
#endif // SAMPLEANDCONTROL_H
