#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

#include "config.h"
#include "rtapi.h"
#include "hal.h"
#include "../src/hal/hal_priv.h"

#define M104_HAL_COMPNAME		"m104-function"

#define M104_HAL_SIG_VAC_REQ_LEFT	"vac-req-left"
#define M104_HAL_SIG_VAC_REQ_RIGHT	"vac-req-right"
#define M104_HAL_SIG_VAC_ACK		"vac-ack"

#define POLL_INTERVAL 20000

static int quit_flag = 0;

static void quit(int sig) {
  quit_flag = 1;
}

static volatile hal_bit_t *get_sig_bit_ptr(const char *name) {
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

  return (volatile hal_bit_t *)SHMPTR(sig->data_ptr);
}


static volatile hal_s32_t *get_sig_s32_ptr(const char *name) {
  hal_sig_t *sig;

  // find signal
  sig = halpr_find_sig_by_name(name);
  if (sig == NULL) {
    return NULL;
  }

  // check datatype
  if (sig->type != HAL_S32) {
    return NULL;
  }

  return (volatile hal_s32_t *)SHMPTR(sig->data_ptr);
}

int main(int argc, char* argv[]) {
  int ret = 1;
  int comp_id;
  int32_t cmd_left;
  int32_t cmd_right;
  volatile hal_s32_t *sig_vac_req_left;
  volatile hal_s32_t *sig_vac_req_right;
  volatile hal_bit_t *sig_vac_ack;

  // check command param
  if (argc < 3) {
    printf("ERROR: command parameter missing!\n");
    goto fail0;
  }

  // get command
  cmd_left = (int)atof(argv[1]);
  cmd_right = (int)atof(argv[2]);

  rtapi_set_msg_level(RTAPI_MSG_ERR);

  signal(SIGINT, quit);
  signal(SIGTERM, quit);
  signal(SIGPIPE, SIG_IGN);

  // init hal
  comp_id = hal_init(M104_HAL_COMPNAME);
  if (comp_id < 0) {
    printf("ERROR: hal init failed!\n");
    goto fail0;
  }
  hal_ready(comp_id);

  // find signals
  rtapi_mutex_get(&(hal_data->mutex));
  if ((sig_vac_req_left = get_sig_s32_ptr(M104_HAL_SIG_VAC_REQ_LEFT)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M104_HAL_SIG_VAC_REQ_LEFT);
    goto fail1;
  }
  if ((sig_vac_req_right = get_sig_s32_ptr(M104_HAL_SIG_VAC_REQ_RIGHT)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M104_HAL_SIG_VAC_REQ_RIGHT);
    goto fail1;
  }
  if ((sig_vac_ack = get_sig_bit_ptr(M104_HAL_SIG_VAC_ACK)) == NULL) {
    rtapi_mutex_give(&(hal_data->mutex));
    printf("ERROR: signal %s not found!\n", M104_HAL_SIG_VAC_ACK);
    goto fail1;
  }
  rtapi_mutex_give(&(hal_data->mutex));

  if (cmd_left > 0 || cmd_right > 0) {
    if ((*sig_vac_req_left != 0) || (*sig_vac_req_right != 0)) {
      printf("ERROR: request for vacuum pod alredy active!\n");
      goto fail1;
    }
    *sig_vac_req_left = cmd_left;
    *sig_vac_req_right = cmd_right;
  } else {
    *sig_vac_req_left = 0;
    *sig_vac_req_right = 0;
  }

  // check ack
  while (!quit_flag) {
    if (((*sig_vac_req_left != 0) || (*sig_vac_req_right != 0)) == *sig_vac_ack) {
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
