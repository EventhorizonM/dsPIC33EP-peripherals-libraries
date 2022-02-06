
/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__dsPIC33E__)
    	#include <p33Exxxx.h>
    #elif defined(__dsPIC33F__)
    	#include <p33Fxxxx.h>
    #endif
#endif


#include <stdint.h>        /* Includes uint16_t definition                    */

#include "System.h"        /* System funct/params, like osc/peripheral config */
#include "LibPWM.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
uint16_t Err;
//--------------------------------------------- LibPWM Related Variables.
unsigned char FaultModePWM1,FaultModePWM2,FaultModePWM3,FaultModePWM4,FaultModePWM5,FaultModePWM6,FaultModePWM7;
/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{
	Setup_OSC();

	Err=SetupPWM_PrimaryMasterTB(PWM_PRESCALER_DIV_1,30000,SEV_POSTSCALER_DIV_1,21000,DISABLE_IMMEDIAT_UPDATE);
//	Err=SetupPWM_SecondaryMasterTB(PWM_PRESCALER_DIV_1,30000,SEV_POSTSCALER_DIV_1,21000,DISABLE_IMMEDIAT_UPDATE);
//	SetupPWMChopGenerator(10, ENABLE_CHOP_CLOCK_GENERATOR);

	/* Intialize PWM Generator 1 */
	Err=SetupPWMGen1(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
	SetupPWMGen1Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
	SetupPWMGen1Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
	SetupPWMGen1PhaseShifting(0, 0);
	Err=SetupPWMGen1DeadTime(POSITIVE_DT_MODE,960,960);
	Err=SetupPWMGen1ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
	Err=SetupPWMGen1Fault(ENABLE_INDEPENDENT_FAULT,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
	Err=SetupPWMGen1CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
	SetupPWMGen1LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
	SetupPWMGen1StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW,PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);

	/* Intialize PWM Generator 2 */
	Err=SetupPWMGen2(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
	SetupPWMGen2Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
	SetupPWMGen2Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
	SetupPWMGen2PhaseShifting(0, 0);
	Err=SetupPWMGen2DeadTime(POSITIVE_DT_MODE,960,960);
	Err=SetupPWMGen2ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
	Err=SetupPWMGen2Fault(ENABLE_INDEPENDENT_FAULT,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
	Err=SetupPWMGen2CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
	SetupPWMGen2LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
	SetupPWMGen2StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW,PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);

	/* Intialize PWM Generator 3 */
	Err=SetupPWMGen3(COMPLEMENTARY_MODE,CENTER_ALIGNED,INDEPENDENT_TIME_BASE,37000,0,INDEPENDENT_DUTY_CYCLE,DISABLE_IMMEDIAT_DC_UPDATE);
	SetupPWMGen3Pins(PWM_MODULE_CONTROL_H_PIN,PWM_MODULE_CONTROL_L_PIN,H_PIN_ACTIVE_HIGH,L_PIN_ACTIVE_HIGH,H_L_PINS_NOT_SWAPED);
	SetupPWMGen3Override(PWM_GEN_CONTROL_H_PIN, PWM_GEN_CONTROL_L_PIN, DONT_CHANGE, DONT_CHANGE, DONT_CHANGE);
	SetupPWMGen3PhaseShifting(0, 0);
	Err=SetupPWMGen3DeadTime(POSITIVE_DT_MODE,960,960);
	Err=SetupPWMGen3ADCTrigger(2400,WAIT_16_PWM_CYC_BEFORE_FIRST_TRIG,TRIGGER_ON_EVERY_TRIGGER_EVENT);
	Err=SetupPWMGen3Fault(ENABLE_INDEPENDENT_FAULT,SOURCE_IS_COMPARATOR3,CYCLE_FAULT_MODE,SET_H_PIN_LOW,SET_L_PIN_LOW,FAULT_INPUT_ACTIVE_LOW);
	Err=SetupPWMGen3CurrentLimit(SOURCE_IS_COMPARATOR2,SET_H_PIN_LOW,SET_L_PIN_LOW,CURRENT_LIM_INPUT_ACTIVE_HIGH);
	SetupPWMGen3LEB(ENABLE_PWMH_RISING_EDGE_BLANK,ENABLE_PWMH_FALLING_EDGE_BLANK,ENABLE_PWML_RISING_EDGE_BLANK,ENABLE_PWML_FALLING_EDGE_BLANK,990);
	SetupPWMGen3StateBlanking(ENABLE_BLANKING_JUST_WHEN_PWMH_IS_HIGH,ENABLE_BLANKING_JUST_WHEN_PWML_IS_LOW,PWM1H_AS_STATE_BLANK_SIGNAL_SOURCE,ENABLE_BLANKING_JUST_WHEN_SIGNAL_IS_HIGH);


	EnableIntPWMSEV();
	SetIntPriorityPWMSEV(4);

	EnableIntPWM1Trig();

	EnableIntPWM1Fault();
	EnablePWM1Fault();
	EnablePWM2Fault();
	EnablePWM3Fault();

	EnableIntPWM1CLim();
	EnablePWM1CurrentLim();
	EnablePWM2CurrentLim();
	EnablePWM3CurrentLim();

	EnableIntPWM1();
	SetIntPriorityPWM1(3);

	EnablePWM1FaultBlanking();
	EnablePWM2FaultBlanking();
	EnablePWM3FaultBlanking();
	
	EnablePWM1CurrentLimBlanking();
	EnablePWM2CurrentLimBlanking();
	EnablePWM3CurrentLimBlanking();

	SetDutyCyclePWM1(15000);
	SetDutyCyclePWM2(10000);
	SetDutyCyclePWM3(5000);

	while(1)
	{
		Nop();
	}
    
}

/******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _PWMSpEventMatchInterrupt(void) {
	_PSEMIF = 0;			// Clear PWM Interuupt.
}

void __attribute__((interrupt, no_auto_psv)) _PWM1Interrupt(void) {
	_PWM1IF = 0;			// Clear PWM Interuupt.
}