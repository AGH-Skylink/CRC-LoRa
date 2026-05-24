/* PID controller port */
#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include "main.h"

typedef struct {
  uint8_t first_call;
  uint32_t prev_time;
  double delta_t;
  double error;
  double prev_error;
  double total_error;
  double P_term;
  double I_term;
  double D_term;
  double pid_output;
  double *angle_ptr;
} PID_HandleTypedef;

void PID_Init(PID_HandleTypedef *h, double *angle_variable);
double PID_Compute(PID_HandleTypedef *h);
double PID_GetCorrection(PID_HandleTypedef *h);
double PID_GetP(PID_HandleTypedef *h);
double PID_GetI(PID_HandleTypedef *h);
double PID_GetD(PID_HandleTypedef *h);

#endif /* INC_PID_H_ */
