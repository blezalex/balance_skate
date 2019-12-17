#ifndef _PID_H
#define _PID_H

#include "common.h"
#include "proto/protocol.pb.h"

class PidController {
public:
	PidController(Config_PidConfig* params)
	{
		_params = params;
	    reset();
	}

	float compute(float error)
	{
		return compute(error, error - _prevError);
	}

	float compute(float error, float de)
	{
		float out = error * _params->p + de * _params->d + _sumI * _params->i;

		_prevError = error;
		_sumI = constrain(_sumI + error, -_params->max_i, _params->max_i);

		return out;
	}

	void reset()
	{
		_sumI = 0;
		_prevError = 0;
	}

public:
	Config_PidConfig *_params;

private:
	float _prevError;
	float _sumI;
};

#endif
