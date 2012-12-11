#include "ch.h"
#include "SampleBuffer.h"

SampleBuffer::SampleBuffer()
  : tp_(0), i_(0)
{
  tp_ = chThdCreateStatic(waWriteThread,
                          sizeof(waWriteThread),
                          NORMALPRIO, WriteThread_, NULL);
}

msg_t SampleBuffer::WriteThread()
{
  chRegSetThreadName("WriteThread");

  while (1) {
    Thread * messaging_tp = chMsgWait();
    msg_t message = chMsgGet(messaging_tp);
    chMsgRelease(messaging_tp, 0);  // return the result of file operation, or zero for increment
    if (message == 0) { // write data in back buffer
        UINT bytes;
        f_write(&f_, reinterpret_cast<uint8_t *>(BackBuffer()),
                sizeof(Sample)*(NUMBER_OF_SAMPLES/2), &bytes);
        f_sync(&f_);
    } else if (message == -1) { // reset the buffer, close file, toss whatever is in the front buffer (implies we lose at most NUMBER_OF_SAMPLES/2)
//      for (i_ = 0; i_ < NUMBER_OF_SAMPLES/2; ++i_) {
//        clearSample(buffer0_[i_]);
//        clearSample(buffer1_[i_]);
//      }
      i_ = 0;
      //f_close(&f_);
    } else {  // open a new file; ASSUMPTION: if the message isn't 0 or -1, it is a pointer to char with the filename.  This should always work as long as you couldn't have a char array located at address 0xFFFFFFFF which is equivalent to -1
      f_open(&f_, reinterpret_cast<const char *>(message), FA_CREATE_ALWAYS | FA_WRITE);
    }
  } // while

  return 0;
}
