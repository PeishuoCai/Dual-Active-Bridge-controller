///*
// * DAB_Iface.c
// *
// *  Created on: Oct. 9, 2023
// *      Author: benma
// */
//
//
//
#include "DAB_Iface.h"
#include <stdio.h>


//2*L*Co*fn/N * 4pi*f_bw
#define kp_theta (2*20e-6*660e-6*100e3/DAB_TURNS_RATIO * 4.0*M_PI*100.0)//0.001f // kp of 0.3 caused oscillations with light load
#define ki_theta (2*20e-6*60e-6*100e3/DAB_TURNS_RATIO * 4.0*M_PI*M_PI*100.0*100.0)//5.0f
#define DAB_CTRL_PERIOD (4000.0f/200000000.0f)// TODO: this
#define thetaMax 1.0f
#define thetaMin DAB_MIN_PHASE_SHIFT
#define DAB_INIT_DURATION 1.0f // seconds
#define DAB_SHUTDOWN_DURATION 1.0f // seconds


#define DAB_RAMP_THETA_STEP thetaMax/(DAB_INIT_DURATION/DAB_CTRL_PERIOD)
#define DAB_SHUTDOWN_WAIT_COUNT (DAB_SHUTDOWN_DURATION/DAB_CTRL_PERIOD)


//uint32_t PS_DAB_WD_counter;
//
char curmode = "debug";
float target_theta = 0;


const float adcscalerDAB = 1/ADC_SCALER_DAB;
const float g_ISEC = 1/G_ISEC;
const float g_TEMP = 1/G_TEMP;
const float g_VSEC = 1/G_VSEC;
const float g_IPRIM = 1/G_IPRIM;
const float g_ITANK = 1/G_ITANK;
const float g_VPRIM = 1/G_VPRIM;

volatile LMG342_GaNFET_DAB_status_t DAB_GaNFETs;


struct DAB_State dab_state;


bool enable_dab =false;

static inline void DAB_checkOverCurrentProtection(void);
static inline void DAB_checkVoltageProtection(void);
static inline HAL_Status_t DAB_checkTemperatureProtection(void);

#define DAB_MAX_DEADBAND_DELAY 16380
#define DAB_STD_DEADBAND_DELAY 20
#define DAB_DEBUG_DEADBAND_DELAY 0
static inline void DAB_setDeadband(uint32_t delay);
static inline void DAB_forceTrip(void);



//
// Triple phase shift control
//

//Calculate primary and secondary duty cycle from vin, vout, and phi
struct Lp phi_prim_slew;
struct Lp phi_sec_slew;



#pragma FUNC_ALWAYS_INLINE(DAB_TPS3)
inline void DAB_TPS3(float32_t vin, float32_t vout, float32_t p)
{
    //D phase shifts all referenced to Primary 1 sync
    float32_t D_1; // Primary duty
    float32_t D_2; // Secondary duty
    float32_t phi; // Phase shift between primary and secondary
    float32_t m = vout*DAB_TURNS_RATIO / vin;


    float32_t absP = fabsf(p);

    //Boost becomes buck
    //if(p<0.0f)
    //    m = 1/m;

    if(m <= 1.0f)
        {


            if( absP < m*(1-m)*0.5 )
            {

                D_1 = sqrtf(2.0f*m*absP/(1.0f-m));
                D_2 = D_1/m;
                phi = 0;
            }
            else if (absP < 0.25f)
            {


                float32_t den = m*m*2.0f - 2.0f*m + 1.0f;

                float32_t m1 = (1.0f - m);

                D_1 = 1.0f - m1*sqrtf((1.0f - 4.0f*absP)/den);
                D_2 = 1.0f;
                phi = (D_1-m)/m1;
            }
            else
            {
                D_1 = 1.0f;
                D_2 = 1.0f;
                phi = 1.0f;

            }
        }
        else
        {
            D_1 = 1.0f;
            D_2 = 1.0f;
            phi = p;
        }

    if(1)//(p>0.0f)
    {
        dab_state.theta_pri = 1.0f - D_1;
        dab_state.theta_sec = phi + (1.0f - D_2);
        dab_state.theta = phi;
    }
    else
    {
        dab_state.theta_pri = -phi - (1.0f - D_2);
        dab_state.theta_sec = D_1 - 1.0f;
        dab_state.theta = -phi;
    }

}

uint16_t prd = DAB_PRIM1_TBPRD;
//https://www.mdpi.com/1996-1073/14/20/6444
//Current stress minimization
#pragma FUNC_ALWAYS_INLINE(DAB_TPS2)
inline void DAB_TPS2(float32_t vin, float32_t vout, float32_t p)
{
    //D phase shifts all referenced to Primary 1 sync
    float32_t D_1; // Primary phase shift
    float32_t D_3; // Secondary phase shift
    float32_t D_2; // Phase shift between primary and secondary
    float32_t period; // Increase pwm for higher power once phase is at maximum
    float32_t m = vout*DAB_TURNS_RATIO / vin;


    //TODO: at max power start updating frequency


    float32_t absp = fabsf(p);

    //Boost becomes buck
    if(p<0.0f)
        m = 1/m;

    if(m <= 1.0f)
        {
            float32_t k = 1.0f/m;

            float32_t k2 = k*k;

            float32_t bound = 2.0f*k-2.0f;

            float32_t tribound = (bound)/k2;

            if( absp <  tribound)
            {
                float32_t f = sqrtf(absp/bound);

                D_1 = 1.0f-f;
                D_2 = (k-1.0f)*f;
                D_3 = D_1;

                period = 1.0f;
            }
            else if (absp < 1.0)
            {

                float32_t den = k2 - bound;

                float32_t f = sqrtf((1-absp)/den);

                D_1 = (k-1.0f)*f;
                D_2 = 0.5f + f*(k-2.0f)*0.5;
                D_3 = D_2;

                period = 1.0f;
            }

            else
            {
                 // Increase counter value
                period = absp;

                D_3 = 0.5;
                 D_2 = 0.5;
                 D_1 = 0.0;
            }



        }
        else
        {

            float32_t m2 = m*m;

            float32_t bound = 2.0f*m-2.0f;

            if( absp < (bound)/m2 )
            {

                float32_t f = sqrtf(absp/bound);

                D_1 = 1.0f - f*m;
                D_2 = 0;
                D_3 = 1.0f - f;

                period = 1.0f;
            }
            else if (absp < 1.0)
            {

                float32_t den = m2 - bound;

                float32_t f = sqrtf((1-absp)/den);

                D_3 = 0.5f - f + 0.5f*m*f;
                D_2 = 0.5f - 0.5f*m * f;
                D_1 = 0;

                period = 1.0f;
            }
            else
            {
                //increase counter value
                period = absp;

                D_3 = 0.5;
                D_2 = 0.5;
                D_1 = 0.0;
            }
        }

    if(p>0.0f)
    {
        dab_state.theta_pri = D_1;
        dab_state.theta_sec = D_3;
        dab_state.theta = D_2;
    }
    else
    {
        dab_state.theta_pri = D_3 - D_2;
        dab_state.theta_sec = D_1 - D_2;
        dab_state.theta = -D_2;

    }

    dab_state.pwmPeriod = prd*period;
    dab_state.CMPAVal = dab_state.pwmPeriod*0.5;


}

//https://ieeexplore.ieee.org/document/5559483
#pragma FUNC_ALWAYS_INLINE(DAB_TPS)
inline void DAB_TPS(float32_t vin, float32_t vout, float32_t p)
{
    float32_t phi_p; // Primary phase shift (max at 1)
    float32_t phi_s; // Secondary phase shift (max at 1)
    float32_t m = vout*DAB_TURNS_RATIO / vin;

    // Bounds on M for stability
    if(m<0.4f)
        m=0.45f;
    if(m>2.0f)
        m=1.5f;

    float32_t absP = fabsf(p);
    float32_t phi = 0;

    //Boost becomes buck
    //if(phi<0.0f)
    //    m = 1/m;

    float32_t signP = (p > 1.0f) - (p < 1.0f);
    phi = signP*sqrtf(absP);

    if(m <= 1.0f)
    {



        if(absP < (1.0f-m) * 0.5f )
        {
            //float32_t den = 1/(1.0-m);
            phi_p = 1.0f - 2.0f*m / (1.0f-m) * phi;
            phi_s = 1.0f - 2.0f / (1.0f-m) * phi;
        }
        else
        {
            // phi max 0.5
            phi_p = (1.0f - 2.0f*phi)*(1.0f-m)/m;
            phi_s = 0.0f;
        }
    }
    else
    {
        float32_t k = 1.0f/m;

        if(absP < (1.0f-k) * 0.5f )
        {

            phi_s = 1.0f - 2.0f*k / (1.0f-k) * phi;
            phi_p = 1.0f - 2.0f / (1.0f-k) * phi;
        }
        else
        {
            // phi max 0.5
            phi_s = (1.0f - 2.0f*phi)*(1.0f-k)/k;
            phi_p = 0.0f;
        }
    }

    // TODO: Lowpass phases to limit slew and avoid oscillations with phi PI controller

    // negative power flow treat secondary as primary
    if(1)//(phi<0.0f)
    {
        dab_state.theta_pri = phi_p;
        dab_state.theta_sec = phi+phi_s;
        dab_state.theta = phi;
    }
    else
    {
        dab_state.theta_pri = phi_s;
        dab_state.theta_sec = phi_p;
    }

    //calcLp(&(dab_state.phi_prim_slew), dab_state.theta_pri);
    //calcLp(&(dab_state.phi_sec_slew), dab_state.theta_sec);

}

inline void DAB_DPS(float32_t vin, float32_t vout, float32_t phi)
{
    float32_t phi_p; // Primary phase shift (max at 1)
     float32_t phi_s; // Secondary phase shift (max at 1)
     float32_t m = vout*DAB_TURNS_RATIO / vin;

     if(phi<0.0f)
         m = 1/m;

     float32_t absPhi = fabsf(phi);

     if(m <= 1.0)
     {
         phi_p = 1.0 - m;
         phi_s = 0;
     }
     else
     {
         phi_p = 0;
         phi_s = 1.0 - 1.0/m;
     }

     // negative power flow treat secondary as primary
     if(phi > 0.0f)
     {
         dab_state.theta_pri = phi_p;
         dab_state.theta_sec = phi_s;
     }
     else
     {
         dab_state.theta_pri = phi_s;
         dab_state.theta_sec = phi_p;
     }

     calcLp(&(dab_state.phi_prim_slew), dab_state.theta_pri);
     calcLp(&(dab_state.phi_sec_slew), dab_state.theta_sec);

}

//// Function to process the SOCA measurements
#pragma FUNC_ALWAYS_INLINE(DAB_SOCA_MEAS)
inline void DAB_SOCA_MEAS(void)
{
    //IPRIM
    dab_state.meas.IPRIM = SCALE_IPRIM(MEAS_DAB_IPRIM_SOCA);

    //ISEC
    dab_state.meas.ISEC = SCALE_ISEC(MEAS_DAB_ISEC_SOCA);

    //VSEC
    dab_state.meas.VSEC = SCALE_VSEC(MEAS_DAB_VSEC_SOCA);

    //IPRIM_TANK
    dab_state.meas.IPRIM_TANK = SCALE_ITANK(MEAS_DAB_IPRIM_TANK_SOCA);

    //ISEC_TANK
    dab_state.meas.ISEC_TANK = SCALE_ITANK(MEAS_DAB_ISEC_TANK_SOCA);

    //VSEC
    dab_state.meas.VPRIM = SCALE_VPRIM(MEAS_DAB_VPRIM_SOCA);
}

// Function to process the SOCB measurements
inline void DAB_SOCB_MEAS(void)
{

    //IPRIM
    dab_state.meas.IPRIM = SCALE_IPRIM(MEAS_DAB_IPRIM_SOCB);

    //ISEC
    dab_state.meas.ISEC = SCALE_ISEC(MEAS_DAB_ISEC_SOCB);

    //VSEC
    dab_state.meas.VSEC = SCALE_VSEC(MEAS_DAB_VSEC_SOCB);

    //IPRIM_TANK
    dab_state.meas.IPRIM_TANK = SCALE_ITANK(MEAS_DAB_IPRIM_TANK_SOCB);

    //ISEC_TANK
    dab_state.meas.ISEC_TANK = SCALE_ITANK(MEAS_DAB_ISEC_TANK_SOCB);

    //VSEC
    dab_state.meas.VPRIM = SCALE_VPRIM(MEAS_DAB_VPRIM_SOCB);
}


__attribute__((ramfunc)) __interrupt void DAB_SOCA_ISR(void)
{
    DAB_SOCA_MEAS();

    //LMG342_GaNFET_DAB_getFaults(&DAB_GaNFETs); // TODO: check speed with and without fault. Is it ok to be slow if fault?

    //PS_DAB_serviceWatchdog();

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_ADCA_BASE_1_INTERRUPT_ACK_GROUP);
}
//
__attribute__((ramfunc)) __interrupt void DAB_SOCB_ISR(void)
{
    //DAB_SOCB_MEAS();

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER2);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INT_ADCA_BASE_2_INTERRUPT_ACK_GROUP);
}

// Frequency update outside of the main control loop
// If developing variable frequency control refer to https://www.ti.com/lit/an/spracy1/spracy1.pdf?ts=1708204313648 for global one shot load documentation
void DAB_setOperatingFrequency(float32_t freq)
{
    //Minumum operating frequency
    if(freq < 50e3)
    {
        freq = 50e3;
    }

    // Clock dividers set to 1
    float32_t pwmclk = 100e6;

    uint16_t countValue = pwmclk/freq/2.0f; //divide by 2 for updown count
    uint16_t cmpaVal = countValue/2.0f; //Half the count value

    //update the registers
    EPWM_setTimeBasePeriod(DAB_PRIM1_BASE, countValue);
    EPWM_setCounterCompareValue(DAB_PRIM1_BASE, EPWM_COUNTER_COMPARE_A, cmpaVal);

}


HAL_Status_t DAB_Startup(void){
    if(dab_state.state == DABState_Idle){

        //DAB_enablePWM();
        //Start with maximum deadband
        DAB_disablePWM();

        dab_state.pwmPeriod = DAB_PRIM1_TBPRD;
        dab_state.CMPAVal = dab_state.pwmPeriod*0.5;

        EPWM_setTimeBasePeriod(DAB_PRIM1_BASE, dab_state.pwmPeriod);
        EPWM_setCounterCompareValue(DAB_PRIM1_BASE, EPWM_COUNTER_COMPARE_A, dab_state.CMPAVal);


        PIon(&dab_state.pi);

        //Initialize theta for full duty cycle
        dab_state.theta_pri = 0.0;
        dab_state.theta_sec = 0.0;

        init_Lp(&(dab_state.phi_prim_slew), 100, DAB_CTRL_PERIOD);
        init_Lp(&(dab_state.phi_sec_slew), 100, DAB_CTRL_PERIOD);

        // start ramp with minumum phase shift
        dab_state.pi.Ymax = thetaMin;
        dab_state.pi.Ymin = thetaMin;

        //Scale gains
        dab_state.pi.ki = ki_theta/(fabsf(dab_state.vprim_lp.state) + 1)*DAB_CTRL_PERIOD;
        dab_state.pi.kp = kp_theta/(fabsf(dab_state.vprim_lp.state) + 1);


        dab_state.state = DABState_StartUp;

        return HAL_SUCCESS;
    } else {
        return HAL_ERROR;
    }
}

void DAB_Shutdown(void){
    if(dab_state.state == DABState_StartUp || dab_state.state == DABState_Running ){
        PIoff(&dab_state.pi);

        dab_state.state = DABState_ShutDown;
        dab_state.shutdown_timer = 0;
    }
}

void DAB_Running(void){
    if (curmode == "debug" && dab_state.stat DABState_StartUp){
        dab_state.state = DABState_Running;
    }
}

void DAB_GetReadings(void){
    printf("IPRIM: %.4f\n", dab_state.meas.IPRIM)
    printf("ISEC: %.4f\n", dab_state.meas.ISEC)
    printf("VSEC: %.4f\n", dab_state.meas.VSEC)
    printf("IPRIM_TANK: %.4f\n", dab_state.meas.IPRIM_TANK)
    printf("ISEC_TANK: %.4f\n", dab_state.meas.ISEC_TANK)
    printf("VPRIM: %.4f\n", dab_state.meas.VPRIM)
}


void DAB_SetVref(float32_t vref)
{

    //TODO: Calulate duty cycle from vin vout ratio

    dab_state.vsec_ref = vref;

}

static inline void DAB_setDeadband(uint32_t delay){
    // Set deadtime. For some reason, the driverlib functions don't work
    HWREGH(DAB_PRIM1_BASE + EPWM_O_DBRED) = delay;
    HWREGH(DAB_PRIM2_BASE + EPWM_O_DBRED) = delay;
    HWREGH(DAB_SEC1_BASE + EPWM_O_DBRED) = delay;
    HWREGH(DAB_SEC2_BASE + EPWM_O_DBRED) = delay;

    HWREGH(DAB_PRIM1_BASE + EPWM_O_DBFED) = delay;
    HWREGH(DAB_PRIM2_BASE + EPWM_O_DBFED) = delay;
    HWREGH(DAB_SEC1_BASE + EPWM_O_DBFED) = delay;
    HWREGH(DAB_SEC2_BASE + EPWM_O_DBFED) = delay;
}

void DAB_disablePWM(void){
   dab_state.deadband = DAB_MAX_DEADBAND_DELAY;
   DAB_setDeadband(dab_state.deadband);
}

void DAB_enablePWM(void){
    dab_state.deadband = DAB_STD_DEADBAND_DELAY;
    DAB_setDeadband(dab_state.deadband);
}

void DAB_enablePWM_simultaneous(void){
    dab_state.deadband = DAB_DEBUG_DEADBAND_DELAY;
    DAB_setDeadband(dab_state.deadband);
}

void DAB_ManualPhaseShift(float desired_theta){
    target_theta = desired_theta
}

static inline void DAB_forceTrip(void){
    EPWM_forceTripZoneEvent(DAB_PRIM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DAB_PRIM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DAB_SEC1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(DAB_SEC2_BASE, EPWM_TZ_FORCE_EVENT_OST);
}

float ki_current = 0.005;

inline float32_t PIcalcff(struct PI*restrict pi, float32_t ref, float32_t u, float32_t ff)
{
    //Calculate error
    pi->error = ref - u;
    pi->error *= pi->enable;

    // Integrator
    float In = pi->error*pi->ki;
    if(pi->sat == POSITIVE_SATURATION)
    {
        if(In < 0)
        {
            pi->I += In;
        }
    }
    else if(pi->sat == NEGATIVE_SATURATION)
    {
        if(In > 0)
        {
            pi->I += In;
        }
    }
    else
    {
        pi->I += pi->error*pi->ki;
    }

    pi->I *= pi->enable; // Set integrator to 0 when disabled

    //Calculate output
    pi->y = pi->I + pi->error*pi->kp + ff * ki_current;

    // check for saturation
    if(pi->y>pi->Ymax)
    {
        pi->y = pi->Ymax;
        pi->sat = POSITIVE_SATURATION;
    }
    else if(pi->y<pi->Ymin)
    {
        pi->y = pi->Ymin;
        pi->sat = NEGATIVE_SATURATION;
    }
    else
    {
        pi->sat = NO_SATURATION;
    }

    return pi->y;

}

uint16_t start_log = false;
uint32_t log_prescale = 0;
uint32_t lut_count = 0;
#pragma FUNC_ALWAYS_INLINE(DAB_ControlISR)
__attribute__((ramfunc)) inline void DAB_ControlISR(void)
{

    //DAB_SOCA_MEAS();
        //LMG342_GaNFET_DAB_getFaults_fast(&DAB_GaNFETs);
    calcLp(&dab_state.vprim_lp, dab_state.meas.VPRIM);

    switch(dab_state.state){
        case DABState_StartUp:

            //Ramp deadtime on primary
            if(dab_state.deadband>DAB_STD_DEADBAND_DELAY)
            {
                DAB_setDeadband(dab_state.deadband);
                dab_state.deadband -= 1;
            }
            else
            {
                //Set standard deadtime
                DAB_enablePWM();

                
                // Ramp phase shift
                if(dab_state.pi.Ymax < thetaMax){
                    dab_state.pi.Ymax += DAB_RAMP_THETA_STEP;
                    dab_state.pi.Ymin -= DAB_RAMP_THETA_STEP;
                } else {
                    // if ramp is done, move to DABState_Running
                    dab_state.pi.Ymax = thetaMax;
                    dab_state.pi.Ymin = -thetaMax;
                    dab_state.state = DABState_Running;
                }
            }

            float32_t pwr = PIcalc(&dab_state.pi, dab_state.vsec_ref, dab_state.meas.VSEC);

            //Update phase shifts
           //Enable triple
           DAB_TPS2(dab_state.vprim_lp.state, dab_state.vsec_ref, pwr); //0.001467f* /dab_state.vprim_lp.state Normalization term 2*L*Co*fn/(N*Vin)



            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;

        case DABState_Running:
        {

            float32_t pwr = PIcalcff(&dab_state.pi, dab_state.vsec_ref, dab_state.meas.VSEC, ps_control.i_meas - dab_state.meas.ISEC);

            if(dab_state.theta > thetaMax){
                dab_state.theta = thetaMax;
            } else if(dab_state.theta < -thetaMax){
                dab_state.theta = -thetaMax;
            }

            //Update phase shifts
           //Enable triple
           DAB_TPS2(dab_state.vprim_lp.state, dab_state.vsec_ref, pwr); //0.001467f* /dab_state.vprim_lp.state Normalization term 2*L*Co*fn/(N*Vin)



             //dab_state.theta = pwr;
            //dab_state.theta_sec = pwr;
            //dab_state.theta_pri = 0;

            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;
        }

        case DABState_ShutDown:
            if(dab_state.theta > thetaMin + DAB_RAMP_THETA_STEP){
                dab_state.theta -= DAB_RAMP_THETA_STEP;
            } else if (dab_state.theta < -thetaMin - DAB_RAMP_THETA_STEP){
                dab_state.theta += DAB_RAMP_THETA_STEP;
            } else {
                dab_state.theta = -thetaMin;
                dab_state.shutdown_timer ++;
                if(dab_state.shutdown_timer > DAB_SHUTDOWN_WAIT_COUNT){
                    // back to idle
                    dab_state.state = DABState_Idle;
                    DAB_disablePWM();
                }
            }
            // ramp to 0 phase shift, run for ~1 second
            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;

        default:
            dab_state.theta = thetaMin;
            break;
    }


    // Update operating frequency
    EPWM_setTimeBasePeriod(DAB_PRIM1_BASE, dab_state.pwmPeriod);
    EPWM_setCounterCompareValue(DAB_PRIM1_BASE, EPWM_COUNTER_COMPARE_A, dab_state.CMPAVal);

    //Update phase shift registers
    DAB_setPhaseShift_Base(DAB_PRIM2_BASE, dab_state.theta_pri, dab_state.CMPAVal, dab_state.pwmPeriod);
    DAB_setPhaseShift_Base(DAB_SEC1_BASE, dab_state.theta, dab_state.CMPAVal, dab_state.pwmPeriod);
    DAB_setPhaseShift_Base(DAB_SEC2_BASE, dab_state.theta_sec, dab_state.CMPAVal, dab_state.pwmPeriod);

    //DAB_setPhaseShift(dab_state.theta);
    // Set global one shot latch (enables DAB_PRIM1 sync out oneshot)
    EPWM_setGlobalLoadOneShotLatch(DAB_PRIM1_BASE);
    EPWM_startOneShotSync(DAB_PRIM1_BASE);






}

// potential code to debug through UART
// int fputc(int ch, FILE *f)
// {
//     while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {} // Wait for space in TX buffer
//     SciaRegs.SCITXBUF.all = ch;
//     return ch;
// }


#pragma FUNC_ALWAYS_INLINE(DAB_ManualISR)
__attribute__((ramfunc)) inline void DAB_ManualISR(void)
{
    // ISR to handle debug functionalities

    // in case of start up, we handle it like normally
    switch(dab_state.state){
        case DABState_StartUp:
            // initial test case want to have simultaneous PWM's outputted without worrying about state of FETs
            printf("starting zero deadband -- make sure input is zero-current")
            //Ramp deadtime on primary
            if(dab_state.deadband>DAB_DEBUG_DEADBAND_DELAY)
            {
                DAB_setDeadband(dab_state.deadband);
                dab_state.deadband -= 1;
            }
            else
            {
                DAB_enablePWM_simultaneous();
                if (dab_state.theta != 0){
                    dab_state.theta = 0;
                }
                // this should start PWM's without delay, since phase shift is 0
            }

            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;

        case DABState_Running:
            // right now it's running simultaneous PWM

            //Ramp deadtime on primary
            if(dab_state.deadband>DAB_STD_DEADBAND_DELAY)
            {
                DAB_setDeadband(dab_state.deadband);
                dab_state.deadband -= 1;
            }
            else
            {
                //Set standard deadtime
                DAB_enablePWM();

                // Ramp phase shift
                float diretion;
                direction = (target_theta - dab_state.theta) / abs(target_theta - dab_state.theta)
                if(dab_state.theta != target_theta){
                    dab_state.theta += diretion * DAB_RAMP_THETA_STEP;
                }
            }

            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;
        
        case DABState_ShutDown:
            if(dab_state.theta > thetaMin + DAB_RAMP_THETA_STEP){
                dab_state.theta -= DAB_RAMP_THETA_STEP;
            } else if (dab_state.theta < -thetaMin - DAB_RAMP_THETA_STEP){
                dab_state.theta += DAB_RAMP_THETA_STEP;
            } else {
                dab_state.theta = -thetaMin;
                dab_state.shutdown_timer ++;
                if(dab_state.shutdown_timer > DAB_SHUTDOWN_WAIT_COUNT){
                    // back to idle
                    dab_state.state = DABState_Idle;
                    DAB_disablePWM();
                }
            }
            // ramp to 0 phase shift, run for ~1 second
            DAB_checkOverCurrentProtection();
            DAB_checkVoltageProtection();
            break;

        default:
            dab_state.theta = thetaMin;
            break;
    // Update operating frequency
    EPWM_setTimeBasePeriod(DAB_PRIM1_BASE, dab_state.pwmPeriod);
    EPWM_setCounterCompareValue(DAB_PRIM1_BASE, EPWM_COUNTER_COMPARE_A, dab_state.CMPAVal);

    //Update phase shift registers
    DAB_setPhaseShift_Base(DAB_PRIM2_BASE, dab_state.theta_pri, dab_state.CMPAVal, dab_state.pwmPeriod);
    DAB_setPhaseShift_Base(DAB_SEC1_BASE, dab_state.theta, dab_state.CMPAVal, dab_state.pwmPeriod);
    DAB_setPhaseShift_Base(DAB_SEC2_BASE, dab_state.theta_sec, dab_state.CMPAVal, dab_state.pwmPeriod);

    //DAB_setPhaseShift(dab_state.theta);
    // Set global one shot latch (enables DAB_PRIM1 sync out oneshot)
    EPWM_setGlobalLoadOneShotLatch(DAB_PRIM1_BASE);
    EPWM_startOneShotSync(DAB_PRIM1_BASE);

}
__attribute__((ramfunc)) __interrupt void INT_DAB_CTRL_ISR(void){
    if (curmode == "default"){
        DAB_ControlISR();
    }
    else if (curmode == "debug") {
        DAB_ManualISR();
    }

    

}

static inline HAL_Status_t DAB_checkTemperatureProtection(void){
    HAL_Status_t result = LMG342_GaNFET_DAB_getTemps(&DAB_GaNFETs);

    if((DAB_GaNFETs.PRIM1_H_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.PRIM1_L_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.PRIM2_H_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.PRIM2_L_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.SEC1_H_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.SEC1_L_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.SEC2_H_status.faults & LMG342_GaNFET_FAULT_OT) ||
        (DAB_GaNFETs.SEC2_L_status.faults & LMG342_GaNFET_FAULT_OT)
    )
    {
        DAB_forceTrip(); // set trip zone
        dab_state.status = DABStatus_OverTemperature;
        dab_state.state = DABState_TripCondition;
    }

    return result;
}

void DAB_hardStop(void){
    DAB_forceTrip(); // set trip zone
    dab_state.status = DABStatus_ExternalFault;
    dab_state.state = DABState_TripCondition;
}

HAL_Status_t DAB_periodicHandler(void){
    HAL_Status_t result;
    if(dab_state.state != DABState_Off){
        result = DAB_checkTemperatureProtection();
        dab_state.meas.Pprim = dab_state.meas.IPRIM * dab_state.meas.VPRIM;
        dab_state.meas.Psec = dab_state.meas.ISEC * dab_state.meas.VSEC;
    }
    
    return result;
}

void DAB_init(void){
    init_PI(&dab_state.pi, kp_theta, ki_theta, thetaMin, -thetaMin, DAB_CTRL_PERIOD);
    init_Lp(&dab_state.vprim_lp, 10, DAB_CTRL_PERIOD);

    dab_state.pwmPeriod = DAB_PRIM1_TBPRD;
    dab_state.CMPAVal = dab_state.pwmPeriod*0.5;

    DAB_disablePWM();
    dab_state.status = DABStatus_Idle;
    dab_state.state = DABState_Off;

    // Configure pwm for oneshot sync

    EPWM_enableGlobalLoadOneShotMode(DAB_PRIM1_BASE);
    EPWM_enableOneShotSync(DAB_PRIM1_BASE);

}

void DAB_setLimits(struct DAB_Limits limits){
    dab_state.limits = limits;

    DAB_setTrip_ISEC_OC(limits.isec_oc);
    DAB_setTrip_IPRIM_OC(limits.iprim_oc);

    // TODO: voltage limits
}

void DAB_setTrip_ISEC_OC(float i_trip){
    int16_t dac_high_val = INV_SCALE_ISEC(i_trip);
    int16_t dac_low_val = INV_SCALE_ISEC(-i_trip);

    if(dac_high_val > DAC_MAX){
        dac_high_val = DAC_MAX;
    } else if (dac_high_val < 0){
        dac_high_val = 0;
    }

    if(dac_low_val > DAC_MAX){
        dac_low_val = DAC_MAX;
    } else if (dac_low_val < 0){
        dac_low_val = 0;
    }

    CMPSS_setDACValueHigh(PS_DAB_ISEC_OC_CMPSS_BASE, (uint16_t) dac_high_val);
    CMPSS_setDACValueLow(PS_DAB_ISEC_OC_CMPSS_BASE, (uint16_t) dac_low_val);
}
void DAB_setTrip_IPRIM_OC(float i_trip){
    int16_t dac_high_val = INV_SCALE_IPRIM(i_trip);
    int16_t dac_low_val = INV_SCALE_IPRIM(-i_trip);

    if(dac_high_val > DAC_MAX){
        dac_high_val = DAC_MAX;
    } else if (dac_high_val < 0){
        dac_high_val = 0;
    }

    if(dac_low_val > DAC_MAX){
        dac_low_val = DAC_MAX;
    } else if (dac_low_val < 0){
        dac_low_val = 0;
    }

    CMPSS_setDACValueHigh(PS_DAB_IPRIM_OC_CMPSS_BASE, (uint16_t) dac_high_val);
    CMPSS_setDACValueLow(PS_DAB_IPRIM_OC_CMPSS_BASE, (uint16_t) dac_low_val);
}

void DAB_clearTrip_ISEC_OC(void){
    CMPSS_clearFilterLatchLow(PS_DAB_ISEC_OC_CMPSS_BASE);
    CMPSS_clearFilterLatchHigh(PS_DAB_ISEC_OC_CMPSS_BASE);
}
void DAB_clearTrip_IPRIM_OC(void){
    CMPSS_clearFilterLatchLow(PS_DAB_IPRIM_OC_CMPSS_BASE);
    CMPSS_clearFilterLatchHigh(PS_DAB_IPRIM_OC_CMPSS_BASE);
}

// TODO: getters
bool DAB_getTrip_ISEC_OC(void){
    return (CMPSS_getStatus(PS_DAB_ISEC_OC_CMPSS_BASE) & (CMPSS_STS_LO_LATCHFILTOUT|CMPSS_STS_HI_LATCHFILTOUT)) != 0;
}
bool DAB_getTrip_IPRIM_OC(void){
    return (CMPSS_getStatus(PS_DAB_IPRIM_OC_CMPSS_BASE) & (CMPSS_STS_LO_LATCHFILTOUT|CMPSS_STS_HI_LATCHFILTOUT)) != 0;
}

static inline void DAB_checkOverCurrentProtection(void){
    //
    // check if epwm is in trip condition due to X-bar inputs (DCAEVT1)
    //
    if( (EPWM_getTripZoneFlagStatus(DAB_PRIM1_BASE) & (EPWM_TZ_FLAG_DCAEVT1) ) != 0)
    {
        //Check comparator latches


        if(DAB_getTrip_ISEC_OC())
        {

            dab_state.status = DABStatus_OverCurrentTrip_Sec;
            dab_state.state = DABState_TripCondition;
            //DAB_clearTrip_ISEC_OC();
        }
        else if (DAB_getTrip_IPRIM_OC())
        {


            dab_state.status = DABStatus_OverCurrentTrip_Prim;
            dab_state.state = DABState_TripCondition;
            //DAB_clearTrip_IPRIM_OC();
        }
        else // if not comparators, then PS_DAB_FAULTn pin
        {
            dab_state.status = DABStatus_FAULTn;
            dab_state.state = DABState_TripCondition;
        }
        ESTOP0;
    }
}

static inline void DAB_checkVoltageProtection(void){
    // VSEC OV
    if(dab_state.meas.VSEC > dab_state.limits.vsec_ov){
        dab_state.vsec_ov_count ++;
        if(dab_state.vsec_ov_count > DAB_VOLTAGE_PROTECTION_DEBOUNCE){
            dab_state.status = DABStatus_OverVoltage_Sec;
            dab_state.state = DABState_TripCondition;
            DAB_forceTrip(); // set trip zone
        }
    } else {
        dab_state.vsec_ov_count = 0;
    }
    if(dab_state.meas.VPRIM > dab_state.limits.vprim_ov){
        dab_state.vprim_ov_count ++;
        if(dab_state.vprim_ov_count > DAB_VOLTAGE_PROTECTION_DEBOUNCE){
            dab_state.status = DAVStatus_OverVoltage_Prim;
            dab_state.state = DABState_TripCondition;
            DAB_forceTrip(); // set trip zone
        }
    } else {
        dab_state.vsec_ov_count = 0;
    }
}

// Attempt to clear faults
bool DAB_clearFaults(void)
{
    if(dab_state.state == DABState_TripCondition || dab_state.state == DABState_Off){
        DAB_disablePWM(); // don't let PWMs start back up when trip zone is cleared

        LMG342_GaNFET_DAB_clearOT(&DAB_GaNFETs);

        //Clear PWM Trips
        EPWM_clearOneShotTripZoneFlag(DAB_PRIM1_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
        EPWM_clearTripZoneFlag(DAB_PRIM1_BASE, EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST);
        EPWM_clearOneShotTripZoneFlag(DAB_PRIM2_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
        EPWM_clearTripZoneFlag(DAB_PRIM2_BASE, EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST);
        EPWM_clearOneShotTripZoneFlag(DAB_SEC1_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
        EPWM_clearTripZoneFlag(DAB_SEC1_BASE, EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST);
        EPWM_clearOneShotTripZoneFlag(DAB_SEC2_BASE, EPWM_TZ_OST_FLAG_DCAEVT1);
        EPWM_clearTripZoneFlag(DAB_SEC2_BASE, EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_OST);

        // Status for checking faults
        dab_state.status = DABStatus_Idle;

        DAB_checkTemperatureProtection();

        DAB_checkVoltageProtection();

        // check over current
        DAB_checkOverCurrentProtection();

        //Check that there is no new fault status

        if(dab_state.status != DABStatus_Idle)
        {
            return false; // if there is a current error
        }


        // No faults
        dab_state.status = DABStatus_NoFault;
        dab_state.state = DABState_Idle;

        return true;
    }

    return false;
}
