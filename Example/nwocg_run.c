#include "nwocg_run.h"
#include <math.h>


static struct
{
	double setpoint;
	double feedback;
	double UnitDelay1;
	double Add1;
	double I_gain;
	double P_gain;
	double Ts;
	double Add2;
	double Add3;
} nwocg;

void nwocg_generated_init()
{
	nwocg.UnitDelay1 = 0;
}

void nwocg_generated_step()
{
	nwocg.Add1 = nwocg.setpoint - nwocg.feedback;
	nwocg.I_gain = nwocg.Add1 * 2;
	nwocg.P_gain = nwocg.Add1 * 3;
	nwocg.Ts = nwocg.I_gain * 0.01;
	nwocg.Add2 = nwocg.Ts + nwocg.UnitDelay1;
	nwocg.Add3 = nwocg.P_gain + nwocg.Add2;

	nwocg.UnitDelay1 = nwocg.Add2;
}

static const nwocg_ExtPort
	ext_ports[] =
{
	{ "command", &nwocg.Add3, 0 },
	{ "setpoint", &nwocg.setpoint, 1 },
	{ "feedback", &nwocg.feedback, 1 },
	{ 0, 0, 0 },
};

const nwocg_ExtPort * const
    nwocg_generated_ext_ports = ext_ports;

const size_t
    nwocg_generated_ext_ports_size = sizeof(ext_ports);
