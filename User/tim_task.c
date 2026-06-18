#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include "types.h"

Sample_struct_t Sample;                                            
Value_struct_t Value;                                              
Control_struct_t Control;  
PID_struct_t vloop;
PID_struct_t ploop;

uint16_t power_start_tick = 0;                                                                                                               //软启动开关
int restart_flag = 0, can_disable_cnt = 0, can_disable_flag = 0;
//0uint32_t tick_200kHz = 0;

void ADC_Linear_calibration(Value_struct_t Value, uint16_t n);                                                                               //不同板子adc校准函数
float ADC_Linear_calibration_init(float p, double real1, double get1, double real2, double get2);                                            //adc线性校准函数
float Fitter(uint16_t *data);                                                                                                                //均值滤波函数
void PI_init(PID_struct_t *pid, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min);                           //pi初始化函数pi参数在main.c
float PI_POSITION(float ref, float set, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min, PID_struct_t *pid);//位置式pi
float PI_DELTA(float ref, float set, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min, PID_struct_t *pid);   //增量式pi
void buckboost_pwm_update(float volt_ratio);                                                                                                 //buck-boost占空比更新函数
void debug_task(void);                                                                                                                       //vofa调试函数

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
    //Control.power_set_target = 50.0f;
    Control.cap_v_max = 25.5f;//平衡电容是10S所以冲到25.5V
	
  if (htim == &htim2)                                                                   //20kHz中断
  {
		Value.vin  = Fitter(Sample.vin)  * adc_conv_fact * V_GAIN;                          //均值滤波+转换
    Value.cin  = (Fitter(Sample.cin)  - I_BIAS) * adc_conv_fact * I_GAIN * R_SAMP_2mR;  //均值滤波+转换
    Value.cout = (Fitter(Sample.cout) - I_BIAS) * adc_conv_fact * I_GAIN * R_SAMP_2mR;  //均值滤波+转换
    Value.vcap =  Fitter(Sample.vcap) * adc_conv_fact * V_GAIN;                         //均值滤波+转换
    Value.ccap = (Fitter(Sample.ccap) - I_BIAS) * adc_conv_fact * I_GAIN * R_SAMP_2mR;  //均值滤波+转换

		ADC_Linear_calibration(Value, 1);                                                   //板子编号传入函数进行校准

		if (Value.vin < 0.1f) Value.vin = 0.1f;                                             //输入电压过小限幅
		if (Value.vcap < 0.1f) Value.vcap = 0.1f;                                           //电容电压过小限幅

		if(power_start_tick < SOFT_START_TIME)                                              //这个if-else为互补对称滤波
    {
			Value.vin = Value.vin;
			Value.vin_last = Value.vin;
			Value.vcap = Value.vcap;
			Value.vcap_last = Value.vcap; 
		}
		else
		{
			Value.vin = 0.1f * Value.vin + 0.9f * Value.vin_last;
			Value.vin_last = Value.vin;
			Value.vcap = 0.1f * Value.vcap + 0.9f * Value.vcap_last;
			Value.vcap_last = Value.vcap;
		}
		
    Value.pin = Value.vin * Value.cin;                                                  //输入功率计算 
    Value.pout = Value.vout * Value.cout;                                                //输出功率计算   
		
    if(power_start_tick < SOFT_START_TIME)                                              //软启动过程
    {
        if(Control.power_set < Control.power_set_target)
            Control.power_set = Control.power_set + 0.01f;
        else
            Control.power_set = Control.power_set - 0.01f;

        power_start_tick++;
    }
    else
    {
        if(Control.power_set < Control.power_set_target - 1.50f)
            Control.power_set = Control.power_set + 1.0f;
        else if(Control.power_set > Control.power_set_target + 1.50f)
            Control.power_set = Control.power_set - 1.0f;
				else 
					  Control.power_set = Control.power_set_target;
    }                                                                                  //
    // 超电也可以这样
		if(Value.vcap < 10.0f){
			if(Control.power_set > 35.0f){
				Control.power_set = 35.0f;
			}
		}
		
    if(Control.power_set > 190.0f)                                                     //这个if-else对power_set限幅
        Control.power_set = 190.0f;
    if(Control.power_set < 10.0f)
        Control.power_set = 10.0f;                                                     //

    //这就是主要的控制算法部分
    Control.ploop_ref = PI_POSITION(Value.pin ,Control.power_set, ploop.kp, ploop.ki, ploop.out_max, ploop.iout_max, ploop.out_min, ploop.iout_min, &ploop);
		Control.vloop_ref = PI_POSITION(Value.vcap ,Control.cap_v_max, vloop.kp, vloop.ki, vloop.out_max, vloop.iout_max, vloop.out_min, vloop.iout_min, &vloop);
		Control.cloop_ref = Control.ploop_ref;
		// 放电时因为ploop_ref算出负的取它，所以ratio取0.005导致直接炸
		
		// 现在加上前馈
		float ffd_ratio = Value.vcap / ((Value.vin < 19.0f) ? 19.0f : Value.vin);
    Control.cloop_ratio = ffd_ratio + Control.cloop_ref;
    Control.vloop_ratio = ffd_ratio + Control.vloop_ref;

		if(Control.ploop_ref > 0.013f) //欠压保护偏置,需要每个超电单独测试                                                         //充电模式
		{

      //双环竞争取小，一开始电流环（其实就是功率环）恒功率，然后电压环恒压动态稳定
			if(Control.vloop_ratio <= Control.cloop_ratio)                                   //电压环优先
			{
				Control.volt_ratio = Control.vloop_ratio;
			}
			else                                                                             //电流环优先
			{
				Control.volt_ratio = Control.cloop_ratio;
			}
      //
		}

		else                                                                                //放电模式
		{
			Control.volt_ratio = Control.cloop_ratio;                                         //放电只用电流环控制

      //欠压保护，防止电容过放，uvp是一个状态位
			if(Value.vcap < 10.0f && Control.uvp == false) 
			{
				Control.uvp = true;
			}
			else if(Value.vcap > 12.0f && Control.uvp ==true)//设定一段回程差
			{
				Control.uvp = false;
			}
			
			if (Control.uvp == true) {
				Control.volt_ratio = 10.5f /Value.vin;
			}
      //
		}
		
		// 限制ratio的最终值,确保离前馈值的范围
		if(Control.volt_ratio > Value.vcap/Value.vin + 0.1f)
		{
			Control.volt_ratio = Value.vcap/Value.vin + 0.1f;
		}
		else if(Control.volt_ratio < Value.vcap / Value.vin - 0.1f)
		{
			Control.volt_ratio = Value.vcap/Value.vin - 0.1f;
		}
    buckboost_pwm_update(Control.volt_ratio);                                          //更新占空比函数调用
    debug_task();                                                                      //vofa调试函数调用
  }
	
	
	if (htim == &htim3)                                                                  //200kHz中断，异常状态保护
  {
    if (HAL_GetTick() - last_can_tick > 500)
        can_disable_cnt++;
    else
        can_disable_cnt = 0;
    if (can_disable_cnt > 5)
        can_disable_flag = 1;  
    else
        can_disable_flag = 0;
		
		  Cap_Com.chassis2cap_msg.robot_hp = 100;
		   //can_disable_flag = 0  ;
		
		
		// 具体的vin需要测试，18可能过于小导致反充
		if(Cap_Com.chassis2cap_msg.robot_hp == 0|| Value.vin < 16.5f || Value.vin > 29.0f || can_disable_flag == 1   )//输入电压异常保护  
    {
      HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);//四开关关闭
      vloop.iout = 0.0f;//积分清空
      ploop.iout = 0.0f;//积分清空
      restart_flag = 1;//重启标志位置1
    }
		if(Cap_Com.chassis2cap_msg.robot_hp != 0 && Value.vin > 17.5f && Value.vin < 27.0f && can_disable_flag == 0 && restart_flag == 1)//输入电压恢复
    {
      HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);//重新开启四开关
      restart_flag = 0;//重启标志位置0
    }
	}

	if (htim == &htim4)                                                                   //10kHz中断，分频到1kHz can发送
  {
		Sample.cnt++;
		if(Sample.cnt == 19)
		{
      Cap_Com.cap2chassis_msg.cap_volt = Value.vcap * Value.vcap / Control.cap_v_max * Control.cap_v_max * 100.0f;
      Cap_Com.cap2chassis_msg.outpower = Value.vin * Value.cout;
      Cap_Com.cap2chassis_msg.cap_volt  = Value.vcap;
			Sample.cnt = 0;
			CAN_Send();
		}
	}
}

void ADC_Linear_calibration(Value_struct_t Value, uint16_t n)//校准函数，第二位为板子编号
{
  switch(n)
  {
    case 0:
    {
    Value.vin  = Value.vin ;
    Value.vcap = Value.vcap;
    Value.ccap = Value.ccap;
    Value.cin  = Value.cin ;
    Value.cout = Value.cout;
    break;
    }
    case 1:
    {
    Value.vin  = ADC_Linear_calibration_init(Value.vin , 25.010, 24.759, 21.010, 20.783);
    Value.vcap = ADC_Linear_calibration_init(Value.vcap, 14.588, 14.298, 11.926, 11.833);
    Value.ccap = ADC_Linear_calibration_init(Value.ccap, 1.531 , 1.453 , 4.966 , 4.983);
    Value.cin  = ADC_Linear_calibration_init(Value.cin , 1.000 , 0.922 , 3.000 , 2.943 );
    Value.cout = ADC_Linear_calibration_init(Value.cout, 0.904 , 0.940 , 3.904 , 4.025 );
    break;
    }
    case 2:
    {
    Value.vin  = ADC_Linear_calibration_init(Value.vin , 25.010, 24.833, 20.010, 19.848);
    Value.vcap = ADC_Linear_calibration_init(Value.vcap, 13.974, 13.950, 10.984, 11.043);
    Value.ccap = ADC_Linear_calibration_init(Value.ccap, 5.004 , 5.900 , 6.370 , 6.826 );
    Value.cin  = ADC_Linear_calibration_init(Value.cin , 3.000 , 3.058 , 3.780 , 3.869 );
    Value.cout = ADC_Linear_calibration_init(Value.cout, 2.820 , 2.841 , 3.544 , 3.553 );
    break;
    }
  }
}

float ADC_Linear_calibration_init(float p, double real1, double get1, double real2, double get2)
{
    double a, b;
    a = (real2 - real1) / (get2 - get1);
    b = (real1 - get1 * a);
    return (float)(p * a + b);
}

float Fitter(uint16_t *data)
{
    uint16_t sum = 0;
    for(int i=0;i<buf_len;i++)
    {
        sum+=data[i];
    }
    return sum * 0.125f;
}

void PI_init(PID_struct_t *pid, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->out_max = out_max;
    pid->iout_max = iout_max;
    pid->out_min = out_min;
    pid->iout_min = iout_min;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->out = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
}

float PI_POSITION(float ref, float set, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min, PID_struct_t *pid)
{
    pid->error[1] = pid->error[0];
    pid->error[0] = set - ref;
    pid->pout = kp * pid->error[0];
    pid->iout += ki * pid->error[0];
    
    pid->iout = (pid->iout > iout_max) ? iout_max : pid->iout;
    pid->iout = (pid->iout < iout_min) ? iout_min : pid->iout;

    pid->out = pid->pout + pid->iout;

    pid->out = (pid->out > out_max) ? out_max : pid->out;
    pid->out = (pid->out < out_min) ? out_min : pid->out;

    return pid->out;
}

void buckboost_pwm_update(float volt_ratio)//支持1-31.6V，在低电压时偏差较大
{
  volt_ratio = (max_volt_ratio < volt_ratio) ? max_volt_ratio : volt_ratio;
  volt_ratio = (min_volt_ratio > volt_ratio) ? min_volt_ratio : volt_ratio;

  if(volt_ratio < 1.0f)//buck模式
  {
    Control.buck_duty = 1 - volt_ratio + 0.032f;//0.032f是修正
    Control.boost_duty = 0.04f;
  }
  else//boost模式
  {
    Control.buck_duty = 0.04f;
    Control.boost_duty =  1- (1 / volt_ratio) + 0.032f;
  }
  Control.buck_duty = (Control.buck_duty > 0.96f) ? 0.96f : Control.buck_duty;
  Control.boost_duty = (Control.boost_duty > 0.96f) ? 0.96f : Control.boost_duty;

  HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER * (1 - Control.buck_duty))/2u;
  HRTIM1->sMasterRegs.MCMP2R = (HRTIM1->sMasterRegs.MPER * (1 + Control.buck_duty))/2u;
  HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER * (1 - Control.boost_duty))/2u;
  HRTIM1->sMasterRegs.MCMP4R = (HRTIM1->sMasterRegs.MPER * (1 + Control.boost_duty))/2u;
}

typedef struct _vofaBuf_t
{
  float data[13];
  char tail[4];
} vofaBuf_t;

vofaBuf_t vofaBuf = {.tail = {0x00, 0x00, 0x80, 0x7F}};
void debug_task(void)
{
    vofaBuf.data[0] = Value.vin;
    vofaBuf.data[1] = Value.cin;
		vofaBuf.data[2] = Control.ploop_ref;
    vofaBuf.data[3] = Value.vcap;
    vofaBuf.data[4] = Value.pin;
    vofaBuf.data[5] = Control.power_set;
    vofaBuf.data[6] = Control.power_set_target;
		vofaBuf.data[7] = Control.cloop_ref;
    vofaBuf.data[8] = Cap_Com.chassis2cap_msg.chassis_power;
    vofaBuf.data[9] = Cap_Com.chassis2cap_msg.pwr_buf;
    vofaBuf.data[10] = Cap_Com.chassis2cap_msg.robot_hp;
    vofaBuf.data[11] = can_disable_flag;
    vofaBuf.data[12] = can_disable_cnt;

  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&vofaBuf, sizeof(vofaBuf));
}