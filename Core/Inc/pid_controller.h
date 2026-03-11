/*
 * pid_controller.h
 *
 *  Created on: Feb 12, 2025
 *      Author:
 */

/* pid_controller.h */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{
	double kp;
	double ki;
	double kd;
	double tau;
	double dt;
	double integrator;
	double differentiator;
	double previous_error;
	double previous_feedback;
	double previous_integrator;
	double output;
	double output_min;
	double output_max;
} PID_t;

void PID_Init_Bartek_s_Lab(PID_t *_pid, double _kp, double _ki, double _kd,
		double _tau, double _output_min, double _output_max);

double PID_Controller_Bartek_s_Lab(PID_t *_pid, double _reference,
		double _feedback, double _dt);

void PI_Adapt_Bartek_s_Lab(PID_t *_pid, double _kp, double _ki);

#endif // PID_CONTROLLER_H
