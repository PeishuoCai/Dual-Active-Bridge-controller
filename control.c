/*
 * control.c
 *
 *  Created on: Jan. 13, 2024
 *      Author: benma
 */

#include "control.h"


//
// Derivative estimator
//

inline void initTD(struct TD* td, float sf, float bw)
{
    td->A[0][0] = 1.0;//1.0-1.0/bw;
    td->A[1][0] = 1.0/sf;
    td->A[0][1] = -bw*bw/sf;
    td->A[1][1] = 1.0 - 2*bw/sf;
    td->B[0] = 0.0;//1.0/bw;
    td->B[1] = bw*bw/sf;
    td->state[0] = 0;
    td->state[1] = 0;
}

inline float TD_calc(struct TD *restrict td, float u)
{
    float temp = td->A[0][0] * td->state[0] + td->A[1][0] * td->state[1] + td->B[0]*u;
    td->state[1] = td->A[0][1] * td->state[0] + td->A[1][1] * td->state[1] + td->B[1]*u;
    td->state[0] = temp;
    return td->state[1];
}


//
// PI control
//
inline void PIoff(struct PI* pi)
{
    pi->enable = 0;
}

inline void PIon(struct PI* pi)
{
    pi->enable = 1;
}


void init_PI(struct PI* pi, float32_t kp, float32_t ki, float32_t Ymax, float32_t Ymin, float32_t ts)
{
    pi->sat = NO_SATURATION; //Initialize with saturation on
    pi->I = 0; //Integrator to 0
    pi->enable = 0; //Default off

    pi->ki = ki*ts; // scale integral gain by sample time
    pi->kp = kp;

    //Saturation limits
    pi->Ymax = Ymax;
    pi->Ymin = Ymin;

}



//Arguments are 1 and -1
inline void setPIsat(struct PI* pi, PI_SAT sat)
{
    pi->sat = sat;
}

inline void setPIresetI(struct PI* pi)
{
    pi->I = 0;
}

inline float32_t PIcalc(struct PI*restrict pi, float32_t ref, float32_t u)
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
    pi->y = pi->I + pi->error*pi->kp;

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


//
// CIC
//

// Order must be power of 2
void CIC_Init(struct CIC* CIC, uint16_t order)
{
    uint32_t R = (1L<<order);
    if (R > (1L<<15))
        R = 1L<<15; //Max gain
    CIC->R = R;
    CIC->inv_DCgain = 1.0/(float)CIC->R;
    CIC_clear(CIC);
}

void CIC_clear(struct CIC* CIC)
{
    CIC->integrator = 0;
    CIC->delay = 0;
    CIC->out=0;
    CIC->r=0;
}


//Input to CIC filter, returns 1 when sample is ready
#pragma FUNC_ALWAYS_INLINE ( CIC_Input )
void CIC_Input(struct CIC*restrict CIC, int32_t input)
{

    CIC->integrator += input;


    CIC->r++;
    if(CIC->r >= CIC->R)
    {
        CIC->r = 0;
        int32_t output = CIC->integrator - CIC->delay;
        CIC->delay = CIC->integrator;

        CIC->out = (float32_t)output*CIC->inv_DCgain;

    }

}


//
// 1st order lp
//


void init_Lp(struct Lp* lp, float32_t fc, float32_t ts)
{
    lp->state = 0;
    float32_t tau = 2*M_PI_f*fc*ts;
    lp->alpha = tau/(tau+1);
}

#pragma FUNC_ALWAYS_INLINE(calcLp)
void calcLp(struct Lp*restrict lp, float32_t u)
{
    lp->state += lp->alpha*(u-lp->state);
}


//
// Biquad filter
//


void biquadReset(biquad* bq)
{
    bq->z[0] = 0.0;
    bq->z[1] = 0.0;
    bq->out = 0.0;
}

void biquadUpdateCoeffs(biquad* bq, const biquadCoefs ref)
{
    bq->coefs = ref;
}


#pragma FUNC_ALWAYS_INLINE ( biquadCalc )
float32_t biquadCalc(float32_t in, biquad*restrict bq)
{
    bq->out = in * bq->coefs.b[0] + bq->z[0];
    bq->z[0] = in * bq->coefs.b[1] + bq->z[1] - bq->coefs.a[0] * bq->out;
    bq->z[1] = in * bq->coefs.b[2] - bq->coefs.a[1] * bq->out;
    bq->out *= bq->coefs.g;
    return bq->out;
}


void calcBiquad_HP(biquad* bq, float32_t Fc, float32_t Fs, float32_t Q) {

    float32_t K = tanf(M_PI_f * Fc / Fs);

    float32_t norm = 1 / (1 + K / Q + K * K);
    bq->coefs.b[0] = 1 * norm;
    bq->coefs.b[1] = -2 * bq->coefs.b[0];
    bq->coefs.b[2] = bq->coefs.b[0];
    bq->coefs.a[0] = 2 * (K * K - 1) * norm;
    bq->coefs.a[1] = (1 - K / Q + K * K) * norm;
    bq->coefs.g = 1.0;

}


void calcBiquad_Notch(biquadCoefs *coeff, float32_t Ts,
                                   float32_t notch_freq,
                                   float32_t c1, float32_t c2)
{
    float32_t temp1;
    float32_t temp2;
    float32_t wn2;
    float32_t Fs;
    Fs = 1/Ts;



    //
    // pre warp the notch frequency
    //
    wn2 = 2 * Fs * tanf(notch_freq * Ts / 2);

    temp1 = 4 * Fs * Fs + 4 * wn2 * c2 * Fs + wn2 * wn2;
    temp2 = 1 / ( 4 * Fs * Fs + 4 * wn2 * c1 * Fs + wn2 * wn2);

    coeff->b[0] = temp1 * temp2;
    coeff->b[1] = (-8 * Fs * Fs + 2 * wn2 * wn2) * temp2;
    coeff->b[2] = (4 * Fs * Fs - 4 * wn2 * c2 * Fs + wn2 * wn2) * temp2;
    coeff->a[0] = (-8 * Fs * Fs + 2 * wn2 * wn2) * temp2;
    coeff->a[1] = (4 * Fs * Fs - 4 * wn2 * c1 * Fs + wn2 * wn2) * temp2;
    coeff->g = 1.0;

}

void calcBiquad_Resonant(biquadCoefs *coeff, float32_t Ts,
                                   float32_t w0,
                                   float32_t wc, float32_t ki)
{

    float32_t Fs;
    Fs = 1/Ts;

    w0 = 2 * Fs * tanf(w0 * Ts / 2);
    float32_t scale = Ts*Ts * w0*w0 + 4*Ts*wc + 4;


    coeff->b[0] = 4*ki*Ts*wc/scale;
    coeff->b[1] = 0*Ts/scale;
    coeff->b[2] = -4*ki*Ts*wc/scale;
    coeff->a[0] = (2 * Ts*Ts * w0*w0 - 8)/scale;
    coeff->a[1] = (Ts*Ts * w0*w0 - 4*Ts*wc + 4)/scale;
    coeff->g = 1.0;

}



//
// SPLL
//

void init_SPLL(struct SPLL* PLL, float32_t f0, float32_t fMin, float32_t fMax, float32_t Kiw, float32_t Kpw, float32_t Kia, float32_t a0, float32_t Ts)
{
    PLL->Amp = a0;
    PLL->Kia = Kia;

    PLL->w0 = f0*2*M_PI_f;
    PLL->wMin = fMin*2*M_PI_f;
    PLL->wMax = fMax*2*M_PI_f;

    PLL->Kiw = Kiw;
    PLL->Kpw = Kpw;

    PLL->sampleTime = Ts;

    PLL->wInteg = 0;
    PLL->lock = 0;
    PLL->theta = 0;

    PLL->delayCount = 0;

    init_Lp(&(PLL->erlp), 1, PLL->sampleTime); //1 hz lowpass
    init_Lp(&(PLL->wlp), 5, PLL->sampleTime); //5 hz lowpass
    init_Lp(&(PLL->alp), 5, PLL->sampleTime); //5 hz lowpass
}



inline float32_t PLLcal_DC( struct SPLL *PLL, float32_t Vin)
{
    PLL->lock = 1;

    calcLp(&(PLL->erlp), Vin);

    PLL->VAl = PLL->erlp.state;

    PLL->sin = 1.0;

    PLL->Amp = fabsf(PLL->erlp.state);

    return PLL->VAl;

}

#pragma FUNC_ALWAYS_INLINE(PLLcal)
inline float32_t PLLcal( struct SPLL *restrict PLL, float32_t Vin)
{
    // Amplitude and frequency errors
    float32_t PllErr, PllErr1;

    // Calculate error
    PllErr = Vin - PLL->VAl;

    // Integral
    PllErr1 = PllErr;

    // Sin portion of the error
    PllErr1*= PLL->sin;

    //Lowpass the synchronous error
    calcLp(&(PLL->erlp), PllErr1);

    // Integrate error
    PllErr1*= PLL->Kia;
    PLL->Amp += PLL->sampleTime * PllErr1;



    // Scale error by estimated amplitude
    PllErr = PllErr / (PLL->Amp+0.001);

    // Calculate cos portion of error (should be 0 when synched to grid)
    PllErr*= PLL->cos;

    // Calculate integral term
    PLL->wInteg+= PLL->sampleTime * PllErr;

    // Update grid frequency estimate
    PLL->w = PLL->wInteg * PLL->Kiw + PllErr * PLL->Kpw + PLL->w0;


    calcLp(&(PLL->wlp), PLL->w);
    calcLp(&(PLL->alp), PLL->Amp);

    // Check for locking
    if(PLL->lock==0 && (PLL->delayCount*PLL->sampleTime > 2) && abs(PLL->erlp.state) < 1) //wait 2s before locking and error should be less than 10V
    {
        // Only connect on the expected frequency range
        if(PLL->w > (PLL->wMin-M_PI_f) && PLL->w < (PLL->wMax+M_PI_f))
        {
            PLL->lock = 1;
            PLL->delayCount = 0;

            // Update limits based on the connected grid frequency
            //if(fabsf(PLL->wlp.state - 50*2*M_PI) < fabsf(PLL->wlp.state - 60*2*M_PI)
            PLL->wMin = PLL->wlp.state-6.2;
            PLL->wMax = PLL->wlp.state+6.2;

        }
    }
    else if (PLL->lock==0)
    {
        PLL->delayCount++;
    }
    else if (PLL->lock==1 && (PLL->w < (PLL->wMin) || PLL->w > (PLL->wMax))) // Disconnect if the frequency goes out of range
    {
        PLL->delayCount++;
        //Debounce
        if(PLL->delayCount > 4000) // 2 cycles at 50Hz 100khz sampling
        {
            //Reset debounce
            PLL->delayCount = 0;
            PLL->lock = 0;
        }
    }
    else
    {
        //Reset debouce
        PLL->delayCount = 0;
    }




    // Update phase
    PLL->theta+= PLL->sampleTime * PLL->w;

    // Phase wraping
    if (PLL->theta >= 6.2832)
        PLL->theta-= 6.2832;

    // Calculate sine portion
    PLL->sin = sinf(PLL->theta);
    PLL->cos = cosf(PLL->theta);

    // Estimate of grid voltage
    PLL->VAl = PLL->Amp * PLL->sin;
    PLL->VBt = (-1) * PLL->Amp * PLL->cos;


    //TODO: Check that PLL has locked to the grid phase

    return PLL->VAl;
}


struct testSin testsin;
void init_testSin(float32_t arms, float32_t f, float32_t phi, float32_t ts)
{
    testsin.a = arms*1.41;
    testsin.f = 2*M_PI_f*f*ts;
    testsin.phi = phi;
    testsin.t = 0;
}

inline float32_t nextSinVal()
{
    testsin.t ++;
    testsin.sin = testsin.a*sinf(testsin.f*testsin.t + testsin.phi);
    return testsin.sin;
}


