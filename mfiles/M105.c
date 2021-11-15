#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "../src/hal/hal_priv.h"

#define M105_HAL_COMPNAME    "m105-function"
#define M105_HAL_SIG_REQUEST "m105-request"

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

  rtapi_set_msg_level(RTAPI_MSG_ERR);

  signal(SIGINT, quit);
  signal(SIGTERM, quit);
  signal(SIGPIPE, SIG_IGN);

  // init hal
  comp_id = hal_init(M105_HAL_COMPNAME);
  if (comp_id < 0) {
    printf("ERROR: hal init failed!\n");
    goto fail0;
  }
  hal_ready(comp_id);

  // find signals
  rtapi_mutex_get(&(hal_data->mutex));
  if ((sig_request = get_sig_bit_ptr(M105_HAL_SIG_REQUEST)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M105_HAL_SIG_REQUEST);
    goto fail1;
  }
  rtapi_mutex_give(&(hal_data->mutex));

  // set request
  *sig_request = 1;
  while (*sig_request) {
    if (quit_flag) {
      goto fail2;
    }
    usleep(POLL_INTERVAL);
  }

  ret = 0;

fail2:
  // reset request
  *sig_request = 0;

fail1:
  // cleanup hal
  hal_exit(comp_id);

fail0:
  return ret;
}
