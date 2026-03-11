/*
 * pid_controller.c
 *
 *  Created on: Feb 12, 2025
 *      Author:
 */

/* pid_controller.c */

#include <pid_controller.h>

void PID_Init_Bartek_s_Lab(PID_t *_pid, double _kp, double _ki, double _kd,
		double _tau, double _output_min, double _output_max)
{
	_pid->kp = _kp;
	_pid->ki = _ki;
	_pid->kd = _kd;
	_pid->tau = _tau;
	_pid->integrator = 0.0;
	_pid->previous_error = 0.0;
	_pid->differentiator = 0.0;
	_pid->previous_feedback = 0.0;
	_pid->previous_integrator = 0.0;
	_pid->output = 0.0;
	_pid->output_min = _output_min;
	_pid->output_max = _output_max;
}

void PI_Adapt_Bartek_s_Lab(PID_t *_pid, double _kp, double _ki)
{
	_pid->kp = _kp;
	_pid->ki = _ki;
}

// https://www.youtube.com/watch?v=NVLXCwc8HzM
double PID_Controller_Bartek_s_Lab(PID_t *_pid, double _reference,
		double _feedback, double _dt)
{
	// Error signal
	double error = _reference - _feedback;

	// Proportional component
	double proportional = _pid->kp * error;

	// Derivative component (band-limited differentiator)
	_pid->differentiator = -(2.0 * _pid->kd
			* (_feedback - _pid->previous_feedback)
			+ (2.0 * _pid->tau - _dt) * _pid->differentiator)
			/ (2.0 * _pid->tau + _dt);

	// Compute output without updating integrator term
	_pid->output = proportional + _pid->previous_integrator
			+ _pid->differentiator;

	if (!(((_pid->output > _pid->output_max) && (error > 0))
			|| ((_pid->output < _pid->output_min) && (error < 0))))
	{
		// Update integral term only if it does not force us deeper into the saturation
		_pid->integrator += 0.5 * _pid->ki * _dt
				* (error + _pid->previous_error);
		_pid->output = proportional + _pid->integrator + _pid->differentiator;
	}

	// Store current integral, error and measurement for the next iteration
	_pid->previous_integrator = _pid->integrator;
	_pid->previous_error = error;
	_pid->previous_feedback = _feedback;

	// Apply output hard limits
	if (_pid->output > _pid->output_max)
	{
		_pid->output = _pid->output_max;
	}
	else if (_pid->output < _pid->output_min)
	{
		_pid->output = _pid->output_min;
	}

	return _pid->output;
}
