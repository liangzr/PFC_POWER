/*
 * main.c
 */

/*    GPIO0,1,2,3  --> LED
 *
 */
/////////////////////////////////////////////////////////
//   Header include
#include "DSP28x_Project.h"


//#pragma CODE_SECTION(EPwm1_timer_isr, "ramfuncs");
//#pragma CODE_SECTION(EPwm2_timer_isr, "ramfuncs");

/////////////////////////////////////////////////////////
//   defined
#define MULTIPLE    1.5
//PID defined
#define PID_DEADBAND	0.1
#define P_GAIN			0.1
#define I_GAIN			0
#define D_GAIN			0
#define INTERGRAL_VAL	0.01

#ifdef TOFLASH
extern void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

#endif

/////////////////////////////////////////////////////////
//   initialite
float sense_volt = 0;
float voltage = 0;
float process_point = 30;
float set_point = 40;
struct _pid
{
	float pv; //integer that contains the process value 过程量
    float sp; //＊integer that contains the set point   设定值
    float integral; // 积分值 －－ 偏差累计值
    float pgain;
    float igain;
    float dgain;
    float deadband;    //死区
    float last_error;
};
struct _pid *pid_x = {0};

interrupt void  cpu_timer0_isr(void);
void pid_tune(struct _pid *pid);
void pid_init (struct _pid *pid, int process_point, int set_point);
float pid_calc (struct _pid *pid);

/////////////////////////////////////////////////////////
//   Main
void main(void)
{
	InitSysCtrl();
	InitGpio();
	InitSciaGpio();
	DINT;
	InitPieCtrl();
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	InitPieVectTable();
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;
	IER |= M_INT1;

	InitAdc();
	ConfigAdc();
	InitEPwm1Gpio();
    InitEPwm();

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 60,100000);
    CpuTimer0Regs.TCR.bit.TSS = 0;              //CpuTimer0 Start/ReStart
    EnableInterrupts();

    pid_tune(pid_x);
    pid_init(pid_x, 0, 15);
#ifdef TOFLASH

	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();
#endif

	while(1)
	{
        sense_volt = (3.3 * (float)AdcResult.ADCRESULT0) / 4096.0;
        pid_x->pv = sense_volt * MULTIPLE;
        EPwm1Regs.CMPA.half.CMPA += pid_calc(pid_x);
        if(EPwm1Regs.CMPA.half.CMPA > 600)
        {
        	EPwm1Regs.CMPA.half.CMPA = 600;
        }
	}
}

interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    //GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

float pid_calc (struct _pid *pid)

{
    int err;
    float pterm, dterm, result, ferror;
    //计算偏差
    err = (pid->sp) - (pid->pv);
    // 判断是否大于死区
    if (abs (err) > pid->deadband)
    {
        ferror = (float) err;   //do integer to float
        // 比例项
        pterm = pid->pgain * ferror;
        if (pterm > 100 || pterm < -100)
        {
            pid->integral = 0.0;
        }
        else
        {
        	// 积分项
            pid->integral += pid->igain * ferror;
            // 输出为0－－100%

            // 如果计算结果大于100，则等于100
            if (pid->integral > 100.0)
            {
                pid->integral = 100.0;
            }

            // 如果计算结果小于0.0，则等于0
            else if (pid->integral < 0.0)
                pid->integral = 0.0;
        }
        // 微分项
        dterm  =  ( (float) (err  -  pid->last_error) )  *  pid->dgain;
        result = pterm + pid->integral + dterm;
    }
    else
        result = pid->integral; // 在死区范围内，保持现有输出
    // 保存上次偏差
    pid->last_error = err;
    // 输出PID值(0-100)
    return (result);
}

void pid_init (struct _pid *pid, int process_point, int set_point)
{
    pid->pv = process_point;
    pid->sp = set_point;
}

void pid_tune(struct _pid *pid)
{
    pid->deadband = PID_DEADBAND;
    pid->pgain = P_GAIN;
    pid->igain = I_GAIN;
    pid->dgain = D_GAIN;
    pid->integral = INTERGRAL_VAL;
}
