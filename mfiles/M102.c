#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "../src/hal/hal_priv.h"

#define M102_HAL_COMPNAME    "m102-function"
#define M102_HAL_SIG_COMMAND "m102-command"
#define M102_HAL_SIG_ENABLE  "m102-enable"

static int quit_flag = 0;

static void quit(int sig) {
  quit_flag = 1;
}

static double *get_sig_float_ptr(const char *name) {
  hal_sig_t *sig;

  // find signal
  sig = halpr_find_sig_by_name(name);
  if (sig == NULL) {
    return NULL;
  }

  // check datatype
  if (sig->type != HAL_FLOAT) {
    return NULL;
  }

  return (double *)SHMPTR(sig->data_ptr);
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
  double cmd;
  int comp_id;
  double *sig_command;
  char *sig_enable;

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
  comp_id = hal_init(M102_HAL_COMPNAME);
  if (comp_id < 0) {
    printf("ERROR: hal init failed!\n");
    goto fail0;
  }
  hal_ready(comp_id);

  // find signals
  rtapi_mutex_get(&(hal_data->mutex));
  if ((sig_command = get_sig_float_ptr(M102_HAL_SIG_COMMAND)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M102_HAL_SIG_COMMAND);
    goto fail1;
  }
  if ((sig_enable = get_sig_bit_ptr(M102_HAL_SIG_ENABLE)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M102_HAL_SIG_ENABLE);
    goto fail1;
  }
  rtapi_mutex_give(&(hal_data->mutex));

  if (cmd > 0.0) {
    *sig_command = cmd;
    *sig_enable = 1;
  } else {
    *sig_enable = 0;
    *sig_command = 0.0;
  }
  ret = 0;

fail1:
  // cleanup hal
  hal_exit(comp_id);

fail0:
  return ret;
}
