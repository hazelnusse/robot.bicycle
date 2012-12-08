#ifndef SAMPLEANDCONTROL_H
#define SAMPLEANDCONTROL_H

#include <cstddef>

#include "ch.h"
#include "ff.h"

#include "Singleton.h"

class SampleAndControl : public Singleton<SampleAndControl> {
  friend class Singleton<SampleAndControl>;
 public:
  void Start(const char * filename = "samples.dat");  // need to implement a way to pass filename const char *);
  void Stop();
  bool isRunning() const;

  static void Control_(void * arg);
  static void shellcmd_(BaseSequentialStream *chp, int argc, char *argv[]);

 private:
  SampleAndControl();
  SampleAndControl(const SampleAndControl &) = delete;
  SampleAndControl & operator=(const SampleAndControl &) = delete;

  void shellcmd(BaseSequentialStream *chp, int argc, char *argv[]);
  void Control();
  void StartCollection();
  void StopCollection();

  WORKING_AREA(waControlThread, 1024);
  // Mailbox mbox_;
  // msg_t messages_[10];
  Thread * tp_;
  bool Running_;
  bool stop_;
};

#include "SampleAndControl_priv.h"
#endif // SAMPLEANDCONTROL_H
