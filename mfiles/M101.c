#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "../src/hal/hal_priv.h"

#define M101_HAL_COMPNAME    "m101-function"
#define M101_HAL_SIG_REQUEST "m101-request"
#define M101_HAL_SIG_READY   "m101-ready"
#define M101_HAL_SIG_ABORT   "m101-abort"

#define POLL_INTERVAL 20000

static int quit_flag = 0;

static void quit(int sig) {
  quit_flag = 1;
}

static char *get_sig_bit_ptr(const char *name) {
  hal_sig_t *sig;

  // find signal
  sig = halpr_find_sig_by_name(name);
  if (sig == NULL) {
    return NULL;
  }

  // check datatype
  if (sig->type != HAL_BIT) {
    return NULL;
  }

  return (char *)SHMPTR(sig->data_ptr);
}

int main() {
  int ret = 1;
  int comp_id;
  char *sig_request;
  char *sig_ready;
  char *sig_abort;

  rtapi_set_msg_level(RTAPI_MSG_ERR);

  signal(SIGINT, quit);
  signal(SIGTERM, quit);
  signal(SIGPIPE, SIG_IGN);

  // init hal
  comp_id = hal_init(M101_HAL_COMPNAME);
  if (comp_id < 0) {
    printf("ERROR: hal init failed!\n");
    goto fail0;
  }
  hal_ready(comp_id);

  // find signals
  rtapi_mutex_get(&(hal_data->mutex));
  if ((sig_request = get_sig_bit_ptr(M101_HAL_SIG_REQUEST)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M101_HAL_SIG_REQUEST);
    goto fail1;
  }
  if ((sig_ready = get_sig_bit_ptr(M101_HAL_SIG_READY)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M101_HAL_SIG_READY);
    goto fail1;
  }
  if ((sig_abort = get_sig_bit_ptr(M101_HAL_SIG_ABORT)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M101_HAL_SIG_ABORT);
    goto fail1;
  }
  rtapi_mutex_give(&(hal_data->mutex));

  // resync
  *sig_request = 0;
  while (*sig_ready || *sig_abort) {
    if (quit_flag) {
      goto fail2;
    }
    usleep(POLL_INTERVAL);
  }

  // set request
  *sig_request = 1;
  while (!(*sig_ready || *sig_abort)) {
    if (quit_flag) {
      goto fail2;
    }
    usleep(POLL_INTERVAL);
  }

  // check for abort
  ret = *sig_abort ? 33 : 32;

fail2:
  // reset request
  *sig_request = 0;

fail1:
  // cleanup hal
  hal_exit(comp_id);

fail0:
  return ret;
}
