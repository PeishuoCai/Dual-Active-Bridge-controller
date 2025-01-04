/*
 * DAB_Iface.h
 *
 *  Created on: Oct. 9, 2023
 *      Author: benma
 */

#ifndef DAB_IFACE_H_
#define DAB_IFACE_H_

#include "syscfg_dupe.h"
#include "device.h"
#include "driverlib.h"
#include "board.h"
#include <LMG342_GaNFET_Iface.h>
#include "PS_Iface.h"
#include <stdbool.h>
#include "control.h"
#include "math.h"


#define DAB_SWITCHING_FREQ ((DEVICE_SYSCLK_FREQ/2) / 1000)

#define DAB_TURNS_RATIO (1.8f) // 18:10 npri:nsec E6551

typedef enum{
    DABStatus_Idle = 0,
    DABStatus_NoFault = 1,
    DABStatus_OverCurrentTrip_Sec = 2,
    DABStatus_OverCurrentTrip_Prim = 4,
    DABStatus_OverVoltage_Sec = 6,
    DAVStatus_OverVoltage_Prim = 7,
    DABStatus_OverTemperature = 8,
    DABStatus_FAULTn = 9,
    DABStatus_ExternalFault = 10
}DAB_deviceStatus;

typedef enum{
    DABState_Off,
    DABState_Idle,
    DABState_StartUp,
    DABState_Running,
    DABState_ShutDown,
    DABState_TripCondition
}DAB_deviceState;


struct DAB_MEAS{

    //ISEC
    volatile float ISEC;

    //VSEC
    volatile float VSEC;

    //IPRIM
    volatile float IPRIM;

    //IPRIM_TANK
    volatile float IPRIM_TANK;

    //ISEC_TANK
    volatile float ISEC_TANK;

    //VPRIM
    volatile float VPRIM;

    //PWR
    float Pprim;
    float Psec;


};

extern volatile LMG342_GaNFET_DAB_status_t DAB_GaNFETs;

#define DAB_VOLTAGE_PROTECTION_DEBOUNCE 8

struct DAB_State{
    struct PI pi;
    struct Lp vprim_lp;
    struct DAB_MEAS meas;
    float vsec_ref;

    uint16_t vsec_ov_count;
    uint16_t vprim_ov_count;

    volatile DAB_deviceStatus status;

    struct DAB_Limits limits;

    volatile DAB_deviceState state;


    // Pri to sec phase shift
    volatile float theta;

    // primary phase shift
    volatile float theta_pri;

    // Secondary phase shift
    volatile float theta_sec;

    // PWM Counter value
    volatile uint16_t pwmPeriod;
    volatile uint16_t CMPAVal;

    struct Lp phi_prim_slew;
    struct Lp phi_sec_slew;

    volatile uint32_t shutdown_timer;


    //Deadband Softstart
    volatile uint32_t deadband;

};

extern struct DAB_State dab_state;

//sensor transfer functions
#define ADC_RES_DAB ADC_RES_PS
#define ADC_REF_DAB ADC_REF_PS
#define DAC_REF_DAB DAC_REF_PS
#define ADC_SCALER_DAB (ADC_RES_DAB/ADC_REF_DAB)
#define DAC_SCALER_DAB (ADC_RES_DAB/DAC_REF_DAB)
#define G_ISEC (41 * (0.006/5.0)*(7.15)/(4.7+4.7))
#define G_TEMP ((ADC_SCALER_DAB));
#define G_VSEC (0.4 * (12.4/(365+365+12.4))*(8.45/(2.55+2.55)))
#define G_IPRIM ((150/10)*(0.01/5.0))
#define G_ITANK (0.02)
#define G_VPRIM (7.15/((3*365)+7.15))

extern const float adcscalerDAB;
extern const float g_ISEC;
extern const float g_TEMP;
extern const float g_VSEC;
extern const float g_IPRIM;
extern const float g_ITANK;
extern const float g_VPRIM;

#define SCALE_ISEC(x) (g_ISEC*(adcscalerDAB*(x - PS_1V65_REF)))
#define INV_SCALE_ISEC(x) (((x * G_ISEC) + ADC_VOFFSET_PS)*DAC_SCALER_DAB)

#define SCALE_IPRIM(x) (g_IPRIM*(adcscalerDAB*(x - PS_1V65_REF)))
#define INV_SCALE_IPRIM(x) (((x * G_IPRIM) + ADC_VOFFSET_PS)*DAC_SCALER_DAB)

#define SCALE_VSEC(x) (g_VSEC*(adcscalerDAB*x))
#define INV_SCALE_VSEC(x) ((x * G_VSEC)*DAC_SCALER_DAB)

#define SCALE_VPRIM(x) (g_VPRIM*(adcscalerDAB*x))
#define INV_SCALE_VPRIM(x) ((x * G_VPRIM)*DAC_SCALER_DAB)

#define SCALE_ITANK(x) (g_ITANK*(adcscalerDAB*(x - PS_1V65_REF)))
#define SCALE_SYSTEM_TEMP(x) ((x-0.5)/(0.01) + 0)


//ADCA EPWM6
#define MEAS_DAB_ISEC_0 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_0))  //SOCA
#define MEAS_DAB_ISEC_1 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_1))  //SOCA
#define MEAS_DAB_ISEC_2 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_2))  //SOCA
#define MEAS_DAB_ISEC_3 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_3))  //SOCA
#define MEAS_DAB_ISEC_4 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_4))  //SOCA
#define MEAS_DAB_ISEC_5 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_5))  //SOCB
#define MEAS_DAB_ISEC_6 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_6))  //SOCB
#define MEAS_DAB_ISEC_7 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_7))  //SOCB
#define MEAS_DAB_ISEC_8 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_8))  //SOCB
#define MEAS_DAB_ISEC_9 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_9))  //SOCB

#define MEAS_DAB_ISEC_SOCA ((float)(MEAS_DAB_ISEC_0 + \
MEAS_DAB_ISEC_1 + \
MEAS_DAB_ISEC_2 + \
MEAS_DAB_ISEC_3 + \
MEAS_DAB_ISEC_4) * 0.2f)

#define MEAS_DAB_ISEC_SOCB ((float)(MEAS_DAB_ISEC_5 + \
MEAS_DAB_ISEC_6 + \
MEAS_DAB_ISEC_7 + \
MEAS_DAB_ISEC_8 + \
MEAS_DAB_ISEC_9) * 0.2f)




//ADCA CPU timer measurements
#define MEAS_SYSTEM_TEMP_1 (((((float)HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_SYSTEM_TEMP_1))*g_TEMP)-0.05)/0.01)


//ADCC EPWM6
#define MEAS_DAB_VSEC_0 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_0)) //SOCA
#define MEAS_DAB_VSEC_1 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_1)) //SOCA
#define MEAS_DAB_VSEC_2 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_2)) //SOCA
#define MEAS_DAB_VSEC_3 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_3)) //SOCA
#define MEAS_DAB_VSEC_4 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_4)) //SOCA
#define MEAS_DAB_VSEC_5 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_5)) //SOCB
#define MEAS_DAB_VSEC_6 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_6)) //SOCB
#define MEAS_DAB_VSEC_7 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_7)) //SOCB
#define MEAS_DAB_VSEC_8 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_8)) //SOCB
#define MEAS_DAB_VSEC_9 (HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_9)) //SOCB

#define MEAS_DAB_VSEC_SOCA ((float) \
(MEAS_DAB_VSEC_0 + \
MEAS_DAB_VSEC_1 + \
MEAS_DAB_VSEC_2 + \
MEAS_DAB_VSEC_3 + \
MEAS_DAB_VSEC_4)  * 0.2f)

#define MEAS_DAB_VSEC_SOCB ((float) \
(MEAS_DAB_VSEC_5 + \
MEAS_DAB_VSEC_6 + \
MEAS_DAB_VSEC_7 + \
MEAS_DAB_VSEC_8 + \
MEAS_DAB_VSEC_9) * 0.2f)

//ADCC EPWM6
#define MEAS_DAB_VPRIM_0 ((float)HWREGH(ADCBRESULT_BASE + (uint32_t)ADCB_BASE_DAB_VPRIM_0)) //SOCA
#define MEAS_DAB_VPRIM_1 ((float)HWREGH(ADCBRESULT_BASE + (uint32_t)ADCB_BASE_DAB_VPRIM_1)) //SOCB

#define MEAS_DAB_VPRIM_SOCA (MEAS_DAB_VPRIM_0)

#define MEAS_DAB_VPRIM_SOCB (MEAS_DAB_VPRIM_1)

//ADCC CPU Timer measurements
#define MEAS_DAB_VSEC_CPU ((float)HWREGH(ADCCRESULT_BASE + (uint32_t)ADCC_BASE_DAB_VSEC_CPU))

//ADCD EPWM1
#define MEAS_DAB_IPRIM_0 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_0))  //SOCA
#define MEAS_DAB_IPRIM_1 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_1))  //SOCA
#define MEAS_DAB_IPRIM_2 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_2))  //SOCA
#define MEAS_DAB_IPRIM_3 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_3))  //SOCA
#define MEAS_DAB_IPRIM_4 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_4))  //SOCA
#define MEAS_DAB_IPRIM_5 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_5))  //SOCB
#define MEAS_DAB_IPRIM_6 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_6))  //SOCB
#define MEAS_DAB_IPRIM_7 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_7))  //SOCB
#define MEAS_DAB_IPRIM_8 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_8))  //SOCB
#define MEAS_DAB_IPRIM_9 (HWREGH(ADCARESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_9))  //SOCB

#define MEAS_DAB_IPRIM_SOCA ((float)(MEAS_DAB_IPRIM_0 + \
MEAS_DAB_IPRIM_1 + \
MEAS_DAB_IPRIM_2 + \
MEAS_DAB_IPRIM_3 + \
MEAS_DAB_IPRIM_4) * 0.2f)

#define MEAS_DAB_IPRIM_SOCB ((float)(MEAS_DAB_IPRIM_5 + \
MEAS_DAB_IPRIM_6 + \
MEAS_DAB_IPRIM_7 + \
MEAS_DAB_IPRIM_8 + \
MEAS_DAB_IPRIM_9) * 0.2f)

#define MEAS_DAB_IPRIM_TANK0 ((float)HWREGH(ADCDRESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_TANK0))  //SOCA
#define MEAS_DAB_IPRIM_TANK1 ((float)HWREGH(ADCDRESULT_BASE + (uint32_t)ADCD_BASE_DAB_IPRIM_TANK1))  //SOCB

#define MEAS_DAB_IPRIM_TANK_SOCA (MEAS_DAB_IPRIM_TANK0)

#define MEAS_DAB_IPRIM_TANK_SOCB (MEAS_DAB_IPRIM_TANK1)

#define MEAS_DAB_ISEC_TANK0 ((float)HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_TANK0))  //SOCA
#define MEAS_DAB_ISEC_TANK1 ((float)HWREGH(ADCARESULT_BASE + (uint32_t)ADCA_BASE_DAB_ISEC_TANK1))  //SOCBS

#define MEAS_DAB_ISEC_TANK_SOCA (MEAS_DAB_ISEC_TANK0)

#define MEAS_DAB_ISEC_TANK_SOCB (MEAS_DAB_ISEC_TANK1)


void DAB_init(void);

/**
 * Sets the phase shift between the primary and secondary of the PFC
 * float phase_shift must be in the range (-0.5, 0.5), which is equivalent to (-pi/2, pi/2)
 */
#define DAB_MAX_PHASE_SHIFT_INT ((DAB_PRIM1_TBPRD/2)-1)
#define DAB_PHASE_SHIFT_DELAY 2 // number of cycles it takes to propogate the sync pulse
#define DAB_MIN_PHASE_SHIFT (1.0/(DAB_PRIM1_TBPRD*2))
static inline void DAB_setPhaseShift(float phase_shift){

    float phase_shift_scaled = ((phase_shift)*(float)(DAB_PRIM1_TBPRD)) - (float)DAB_PHASE_SHIFT_DELAY;
    uint32_t phase_shift_int;
    if(phase_shift_scaled > 0){
        if(phase_shift_scaled > DAB_MAX_PHASE_SHIFT_INT){
            phase_shift_scaled = DAB_MAX_PHASE_SHIFT_INT;
        }
        phase_shift_int = (uint16_t)phase_shift_scaled;

        EPWM_setCountModeAfterSync(DAB_SEC1_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
        EPWM_setCountModeAfterSync(DAB_SEC2_BASE, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
    } else {
        if(phase_shift_scaled < -DAB_MAX_PHASE_SHIFT_INT){
            phase_shift_scaled = -DAB_MAX_PHASE_SHIFT_INT;
        }
        phase_shift_int = (uint16_t)(-phase_shift_scaled);
        EPWM_setCountModeAfterSync(DAB_SEC1_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);
        EPWM_setCountModeAfterSync(DAB_SEC2_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    }
    EPWM_setPhaseShift(DAB_SEC1_BASE, phase_shift_int);
    EPWM_setPhaseShift(DAB_SEC2_BASE, phase_shift_int);


}


#define GET_CMPA(base) ((uint16_t)((HWREG(base + EPWM_O_CMPA) &0xFFFF0000UL) >> 16U))

#define GET_CMPA_UP_ACTION(base)   ((HWREGH(base + EPWM_O_AQCTLA) >> ((uint16_t)EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA)) & 3U)
#define GET_CMPA_DOWN_ACTION(base)   ((HWREGH(base + EPWM_O_AQCTLA) >> ((uint16_t)EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA)) & 3U)



// Phase shift controller for primary and secondary lagging phases
 inline void DAB_setPhaseShift_Base(uint32_t base, float32_t phase_shift, uint16_t cmpaVal, uint16_t period)
{
    // All phase shifts are referenced to phase 1 of primary (epwm 8)

    //Wrap the phase shift
    if(phase_shift>1.0)
        phase_shift = 1.0 - phase_shift; // Equivalent to negative phase vs ph pri 1
    if(phase_shift<-1.0)
        phase_shift = phase_shift-1.0; // Equivalent to posistive phase vs ph pri 1

    float phase_shift_scaled = ((phase_shift)*(float)(period)) - (float)DAB_PHASE_SHIFT_DELAY;
       uint32_t phase_shift_int;


       if(phase_shift_scaled > 0){

           // Set CMPB equal to TBPSH Register value -1
           //Set the down count action to the same as CMPA down count action
           //Do nothing on upcount

           // Get action
           EPWM_ActionQualifierOutput output = GET_CMPA_DOWN_ACTION(base);

            // Clear CMPB upcount event
           EPWM_setActionQualifierAction(base,EPWM_AQ_OUTPUT_A,EPWM_AQ_OUTPUT_NO_CHANGE,
                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);


           if(phase_shift_scaled > DAB_PRIM1_TBPRD){
               phase_shift_scaled = DAB_PRIM1_TBPRD;

           }

           phase_shift_int = (uint16_t)phase_shift_scaled;

           // Set CMPB = CMPA
           uint16_t cmpb = cmpaVal;

           // Check if phase_shift is <= CMPA value
           if(phase_shift_int <= cmpb && phase_shift_int>1)
           {
               // Next count after sync will trigger event
               cmpb = (phase_shift_int - 1);
           }

           EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, cmpb);

           // Set CMPB downcount evet
           EPWM_setActionQualifierAction(base,EPWM_AQ_OUTPUT_A,output,
                                         EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);

           // Set count down on sync
           EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);

       } else {


         //Set the up count action to the same as CMPA down count action
         //Do nothing on down count

         // Get action
         EPWM_ActionQualifierOutput output = GET_CMPA_UP_ACTION(base);



        // Clear CMPB downcount evet
        EPWM_setActionQualifierAction(base,EPWM_AQ_OUTPUT_A,EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);


           if(phase_shift_scaled < -DAB_PRIM1_TBPRD){
               phase_shift_scaled = -DAB_PRIM1_TBPRD;
           }


           phase_shift_int = (uint16_t)(-phase_shift_scaled);

           // Set CMPB = CMPA
           uint16_t cmpb = cmpaVal;

           // Check if phase_shift is >= CMPA value
           if(phase_shift_int >= cmpb && phase_shift_int<DAB_PRIM1_TBPRD)
           {
               // Next count after sync will trigger event
               cmpb = (phase_shift_int + 1);
           }

           EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B, cmpb);

           // Set CMPB upcount event
           EPWM_setActionQualifierAction(base,EPWM_AQ_OUTPUT_A,output,
               EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

           EPWM_setCountModeAfterSync(base, EPWM_COUNT_MODE_UP_AFTER_SYNC);
       }

       //Set the phase shift
       EPWM_setPhaseShift(base, phase_shift_int);

}


/**
 * Startup dab
 */
HAL_Status_t DAB_Startup(void);

/**
 * ShutDown dab
 */
void DAB_Shutdown(void);

void DAB_enablePWM(void);

void DAB_enablePWM_simultaneous(void);

void DAB_disablePWM(void);

bool DAB_clearFaults(void);

void DAB_hardStop(void);

void DAB_SetVref(float32_t vref);

void DAB_setLimits(struct DAB_Limits limits);
void DAB_setTrip_ISEC_OC(float i_trip);
void DAB_setTrip_IPRIM_OC(float i_trip);
void DAB_clearTrip_ISEC_OC(void);
void DAB_clearTrip_IPRIM_OC(void);
bool DAB_getTrip_ISEC_OC(void);
bool DAB_getTrip_IPRIM_OC(void);

void DAB_Running(void);
void DAB_GetReadings(void);
void DAB_ManualPhaseShift(float desired_theta);


/**
 * Should be called periodically, but not in an ISR
 * ATM, should take 3 to 8 ms
 */
HAL_Status_t DAB_periodicHandler(void);

#endif /* DAB_IFACE_H_ */
