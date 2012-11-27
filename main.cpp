#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "evtimer.h"

#include "ff.h"

#include "SampleAndControl.h"
#include "RearWheel.h"

/*===========================================================================*/
/* Card insertion monitor.                                                   */
/*===========================================================================*/

static const int POLLING_INTERVAL = 20;
static const int POLLING_DELAY = 20;

/**
 * @brief   Card monitor timer.
 */
static VirtualTimer tmr;

/**
 * @brief   Debounce counter.
 */
static unsigned cnt;

/**
 * @brief   Card event sources.
 */
static EventSource inserted_event, removed_event;

/**
 * @brief   Insertion monitor timer callback function.
 *
 * @param[in] p         pointer to the @p BaseBlockDevice object
 *
 * @notapi
 */
static void tmrfunc(void *p) {
  BaseBlockDevice *bbdp = static_cast<BaseBlockDevice *>(p);

  chSysLockFromIsr();
  if (cnt > 0) {
    if (blkIsInserted(bbdp)) {
      if (--cnt == 0) {
        chEvtBroadcastI(&inserted_event);
      }
    }
    else
      cnt = POLLING_INTERVAL;
  }
  else {
    if (!blkIsInserted(bbdp)) {
      cnt = POLLING_INTERVAL;
      chEvtBroadcastI(&removed_event);
    }
  }
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, bbdp);
  chSysUnlockFromIsr();
}

/**
 * @brief   Polling monitor start.
 *
 * @param[in] p         pointer to an object implementing @p BaseBlockDevice
 *
 * @notapi
 */
static void tmr_init(void *p)
{
  chEvtInit(&inserted_event);
  chEvtInit(&removed_event);
  chSysLock();
  cnt = POLLING_INTERVAL;
  chVTSetI(&tmr, MS2ST(POLLING_DELAY), tmrfunc, p);
  chSysUnlock();
}

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS SDC_FS;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

static const ShellCommand commands[] = {
  {"speed", RearWheel::shellcmd},     // disable/enable speed control, select set point
//  {"yawrate", YawRateController::shellcmd}, // disable/enable yawrate control, select set point
  {"control_loop", SampleAndControl::chshellcmd},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD2,
  commands
};

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/*
 * MMC card insertion event.
 */
static void InsertHandler(__attribute__((unused)) eventid_t id)
{
  FRESULT err;

  /*
   * On insertion SDC initialization and FS mount.
   */
  if (sdcConnect(&SDCD1))
    return;

  err = f_mount(0, &SDC_FS);
  if (err != FR_OK) {
    sdcDisconnect(&SDCD1);
    return;
  }
  palSetPad(GPIOC, GPIOC_LED);
}

/*
 * MMC card removal event.
 */
static void RemoveHandler(__attribute__((unused)) eventid_t id)
{
  sdcDisconnect(&SDCD1);
  palClearPad(GPIOC, GPIOC_LED);
}

/*
 * Application entry point.
 */
int main()
{
  static const evhandler_t evhndl[] = { InsertHandler, RemoveHandler };
  Thread * shelltp = NULL;
  static struct EventListener el0, el1;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  chRegSetThreadName("main");

  shellInit();            // Shell manager initialization

  sdStart(&SD2, NULL);    // Activate serial driver 2, default configuration

  sdcStart(&SDCD1, NULL); // Activate SDC driver 1, default configuration
  tmr_init(&SDCD1);       // Activates the card insertion monitor.

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and listen for events.
   */
  chEvtRegister(&inserted_event, &el0, 0);
  chEvtRegister(&removed_event, &el1, 1);
  static WORKING_AREA(waShell, 2048);
  while (true) {
    if (!shelltp)
      shelltp = shellCreateStatic(&shell_cfg1, waShell, sizeof(waShell), NORMALPRIO);
    else if (chThdTerminated(shelltp)) {
      shelltp = NULL;           /* Triggers spawning of a new shell.        */
    }
    chEvtDispatch(evhndl, chEvtWaitOne(ALL_EVENTS));
  }
}
