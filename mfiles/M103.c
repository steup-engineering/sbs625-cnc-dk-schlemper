#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "../src/hal/hal_priv.h"

#define M103_HAL_COMPNAME          "m103-function"

#define M103_HAL_SIG_TOOLPROBE_REQ "toolprobe-req"
#define M103_HAL_SIG_MATPROBE_REQ  "matprobe-req"
#define M103_HAL_SIG_MATPROBE_ACK  "matprobe-ack"

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

int main(int argc, char* argv[]) {
  int ret = 1;
  int comp_id;
  double cmd;
  char *sig_toolprobe_req;
  char *sig_matprobe_req;
  char *sig_matprobe_ack;

  // check command param
  if (argc < 2) {
    printf("ERROR: command parameter missing!\n");
    goto fail0;
  }

  // get command
  cmd = atof(argv[1]);

  rtapi_set_msg_level(RTAPI_MSG_ERR);

  signal(SIGINT, quit);
  signal(SIGTERM, quit);
  signal(SIGPIPE, SIG_IGN);

  // init hal
  comp_id = hal_init(M103_HAL_COMPNAME);
  if (comp_id < 0) {
    printf("ERROR: hal init failed!\n");
    goto fail0;
  }
  hal_ready(comp_id);

  // find signals
  rtapi_mutex_get(&(hal_data->mutex));
  if ((sig_toolprobe_req = get_sig_bit_ptr(M103_HAL_SIG_TOOLPROBE_REQ)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M103_HAL_SIG_TOOLPROBE_REQ);
    goto fail1;
  }
  if ((sig_matprobe_req = get_sig_bit_ptr(M103_HAL_SIG_MATPROBE_REQ)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M103_HAL_SIG_MATPROBE_REQ);
    goto fail1;
  }
  if ((sig_matprobe_ack = get_sig_bit_ptr(M103_HAL_SIG_MATPROBE_ACK)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M103_HAL_SIG_MATPROBE_ACK);
    goto fail1;
  }
  rtapi_mutex_give(&(hal_data->mutex));

  if (cmd >= 2.0) {
    // toolprobe request
    *sig_toolprobe_req = 1;
    *sig_matprobe_req = 0;
  } else if (cmd >= 1.0) {
    // matprobe request
    *sig_toolprobe_req = 0;
    *sig_matprobe_req = 1;
  } else {
    // request off
    *sig_toolprobe_req = 0;
    *sig_matprobe_req = 0;
  }

  // check ack
  while (!quit_flag) {
    if (*sig_matprobe_req == *sig_matprobe_ack) {
      ret = 0;
      break;
    }
    usleep(POLL_INTERVAL);
  }

fail1:
  // cleanup hal
  hal_exit(comp_id);

fail0:
  return ret;
}
