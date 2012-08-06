#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"

class SampleAndControl {
 public:
  static SampleAndControl & Instance();
  static void chshellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static msg_t Control(void * arg);
  void Enable();
  bool isEnabled() const;
  void Disable();
  bool isDisabled() const;
  void SetFilename(const char * name);
  char * Filename() const;

 private:
  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  static msg_t WriteThread(void * arg);
  SampleAndControl();
  ~SampleAndControl();
  static void *operator new(std::size_t, void * location);
  static SampleAndControl * instance_;
  Thread * Control_tp_, * Write_tp_;
  bool Enabled_;
  char Filename_[24];
  SampleAndControl & operator=(const SampleAndControl &) = delete;
  SampleAndControl(const SampleAndControl &) = delete;
};

#endif // SAMPLEANDCONTROL_H
