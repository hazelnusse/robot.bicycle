#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"
#include "ff.h"

#include "EncoderTimers.h"

class SampleAndControl {
 public:
  static SampleAndControl & Instance();
  static void chshellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void Control(void * arg) __attribute__ ((noreturn));
  void Enable();
  bool Enabled() const { return Enabled_; }
  void Disable();
  bool Disabled() const { return !Enabled_; }
  void SetFilename(const char * name);
  char * Filename() const;
  static EncoderTimers timers;

 private:
  SampleAndControl();
  ~SampleAndControl();
  SampleAndControl & operator=(const SampleAndControl &) = delete;
  SampleAndControl(const SampleAndControl &) = delete;
  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static void *operator new(std::size_t, void * location);
  static SampleAndControl * instance_;

  Thread * Control_tp_;
  bool Enabled_;
  char Filename_[24];
  static FIL f_;
  static WORKING_AREA(waControlThread, 1024);
};
#endif // SAMPLEANDCONTROL_H
