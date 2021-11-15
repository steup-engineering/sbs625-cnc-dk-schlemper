/********************************************************************
* Description: bridgesawkins.c
*   Trivial kinematics for 3 axis Cartesian machine
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2007 Chris Radek
*
* Last change:
********************************************************************/

#include "kinematics.h"		/* these decls */
#include "posemath.h"
#include "hal.h"
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "rtapi_math.h"

struct haldata {
    hal_float_t *angular_scale;
    hal_float_t *apivot_toolaxis;
    hal_float_t *apivot_bladepuck;
    hal_float_t *cpivot_toolaxis;
    hal_float_t *cpivot_apivot;
    hal_float_t *tooloffset_a;
    hal_float_t *tooloffset_c;
    hal_float_t *tooloffset_u;
    hal_float_t *tooloffset_v;
    hal_float_t *tooloffset_w;
} *haldata;

#define d2r(d) ((d)*PM_PI/180.0)

void pmRotX(PmCartesian *pc, double a)
{
    double y = pc->y, z = pc->z;
    a = d2r(a);
    pc->y = y*cos(a) - z*sin(a);
    pc->z = y*sin(a) + z*cos(a);
}

void pmRotZ(PmCartesian *pc, double a)
{
    double x = pc->x, y = pc->y;
    a = d2r(a);
    pc->x = x*cos(a) - y*sin(a);
    pc->y = x*sin(a) + y*cos(a);
}

void calcBladeTipOffset(PmCartesian *pc, double a, double c)
{
    // build bladepuck center point to b pivot vector
    pc->x = 0;
    pc->y = *(haldata->apivot_bladepuck);
    pc->z = *(haldata->apivot_toolaxis);

    // rotate a (y/z-pane)
    pmRotX(pc, a);
    
    // add a pivot to c pivot vector
    pc->x += *(haldata->cpivot_toolaxis);
    pc->y += *(haldata->cpivot_apivot);

    // rotate c (x/y-pane)
    pmRotZ(pc, c);
}

void calcToolPaneOffset(PmCartesian *pc, double u, double v, double w, double a, double c)
{
    // create v/w vector
    pc->x = u;
    pc->y = v;
    pc->z = w;

    // rotate a (y/z-pane)
    pmRotX(pc, a);

    // rotate c (x/y-pane)
    pmRotZ(pc, c);
}

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    PmCartesian bto;
    PmCartesian tpo;
    double int_a, int_c;

    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    pos->u = joints[6] - *(haldata->tooloffset_u);
    pos->v = joints[7] - *(haldata->tooloffset_v);
    pos->w = joints[8] - *(haldata->tooloffset_w);

    int_a = pos->a * *(haldata->angular_scale);
    int_c = pos->c * *(haldata->angular_scale);

    calcBladeTipOffset(&bto, int_a, int_c);
    calcToolPaneOffset(&tpo, pos->u, pos->v, pos->w, int_a, int_c);

    pos->tran.x = joints[0] + bto.x - tpo.x;
    pos->tran.y = joints[1] + bto.y - tpo.y;
    pos->tran.z = joints[2] + bto.z - tpo.z;

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    PmCartesian bto;
    PmCartesian tpo;
    double int_a, int_c;

    int_a = pos->a * *(haldata->angular_scale);
    int_c = pos->c * *(haldata->angular_scale);

    calcBladeTipOffset(&bto, int_a, int_c);
    calcToolPaneOffset(&tpo, pos->u, pos->v, pos->w, int_a, int_c);

    joints[0] = pos->tran.x - bto.x + tpo.x;
    joints[1] = pos->tran.y - bto.y + tpo.y;
    joints[2] = pos->tran.z - bto.z + tpo.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    joints[6] = pos->u + *(haldata->tooloffset_u);
    joints[7] = pos->v + *(haldata->tooloffset_v);
    joints[8] = pos->w + *(haldata->tooloffset_w);

    return 0;
}

/* implemented for these kinematics as giving joints preference */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
    *fflags = 0;
    *iflags = 0;

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}


EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) {
    int result;
    comp_id = hal_init("bridgesawkins");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));
    if((result = hal_pin_float_new("bridgesawkins.angular-scale", HAL_IN, &(haldata->angular_scale), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.apivot-toolaxis", HAL_IN, &(haldata->apivot_toolaxis), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.apivot-bladepuck", HAL_IN, &(haldata->apivot_bladepuck), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.cpivot-toolaxis", HAL_IN, &(haldata->cpivot_toolaxis), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.cpivot-apivot", HAL_IN, &(haldata->cpivot_apivot), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.tooloffset.a", HAL_IN, &(haldata->tooloffset_a), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.tooloffset.c", HAL_IN, &(haldata->tooloffset_c), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.tooloffset.u", HAL_IN, &(haldata->tooloffset_u), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.tooloffset.v", HAL_IN, &(haldata->tooloffset_v), comp_id)) < 0) goto error;
    if((result = hal_pin_float_new("bridgesawkins.tooloffset.w", HAL_IN, &(haldata->tooloffset_w), comp_id)) < 0) goto error;

    *(haldata->angular_scale) = 1.0;

    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return result;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

