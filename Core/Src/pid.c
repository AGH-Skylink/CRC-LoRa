/* pid.c
 * Simple PID ported from legacy Arduino implementation
 */

#include "pid.h"
#include "main.h"

/* Tunable constants */
extern double KP = 1.0;
extern double KI = 0.0;
extern double KD = 0.0;
extern double PID_SETPOINT;
extern double PID_DEG_MAX;
extern double PID_DEG_MIN;

void PID_Init(PID_HandleTypedef *h, double *angle_variable) {
  h->first_call = 1;
  h->prev_time = 0;
  h->delta_t = 0.0;
  h->error = 0.0;
  h->prev_error = 0.0;
  h->total_error = 0.0;
  h->P_term = 0.0;
  h->I_term = 0.0;
  h->D_term = 0.0;
  h->pid_output = 0.0;
  h->angle_ptr = angle_variable;
}

double PID_Compute(PID_HandleTypedef *h) {
  double angle = *(h->angle_ptr);
  if (h->first_call) {
    h->first_call = 0;
    h->prev_time = HAL_GetTick();
    h->prev_error = angle - PID_SETPOINT;
    h->pid_output = 0.0;
  } else {
    uint32_t time = HAL_GetTick();
    h->delta_t = (time - h->prev_time) / 1000.0; // seconds
    h->error = angle - PID_SETPOINT;
    h->P_term = h->error;
    h->I_term += h->error * h->delta_t;
    if (h->delta_t > 0.0)
      h->D_term = (h->error - h->prev_error) / h->delta_t;
    if (((h->error < 0) != (h->prev_error < 0)) || (h->error == 0)) {
      h->I_term = 0.0;
    }
    h->prev_error = h->error;
    h->prev_time = time;
    h->pid_output = (KP * h->P_term) + (KI * h->I_term) + (KD * h->D_term);
    if (h->pid_output > PID_DEG_MAX) h->pid_output = PID_DEG_MAX;
    else if (h->pid_output < PID_DEG_MIN) h->pid_output = PID_DEG_MIN;
  }
  return h->pid_output;
}

double PID_GetCorrection(PID_HandleTypedef *h) { return h->pid_output; }
double PID_GetP(PID_HandleTypedef *h) { return h->P_term; }
double PID_GetI(PID_HandleTypedef *h) { return h->I_term; }
double PID_GetD(PID_HandleTypedef *h) { return h->D_term; }
