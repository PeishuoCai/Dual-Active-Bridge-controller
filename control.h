/*
 * control.h
 *
 *  Created on: Jan. 13, 2024
 *      Author: benma
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "device.h"
#include "math.h"

#define M_PI_f        3.14159265358979323846f  /* pi */
#define M_PI_2_f      1.57079632679489661923f  /* pi/2 */
#define M_SQRT2_f     1.41421356237309504880f  /* sqrt(2) */
#define M_SQRT1_2_f   0.70710678118654752440f  /* 1/sqrt(2) */



typedef enum{
    NEGATIVE_SATURATION = -1,
    NO_SATURATION = 0,
    POSITIVE_SATURATION = 1,
}PI_SAT;

struct PI
{

    float32_t kp; // Proportional gain
    float32_t ki; // Integrator gain
    float32_t I; // Integrator
    uint16_t enable; // Enable integration
    float32_t Ymax; // anti wind up upper threshold
    float32_t Ymin; // anti wind up lower threshold
    PI_SAT sat; // Saturation tracking for anti windup

    float32_t error;
    float32_t y; //output

};



//
// Derivative estimator
//


struct TD{
    float A[2][2];
    float B[2];
    float state[2];
};
inline void initTD(struct TD* td, float sf, float bw);
inline float TD_calc(struct TD *restrict td, float u);



inline void PIoff(struct PI* pi);
inline void PIon(struct PI* pi);
void init_PI(struct PI* pi, float32_t kp, float32_t ki, float32_t Ymax, float32_t Ymin, float32_t ts);
inline float32_t PIcalc(struct PI* pi, float32_t ref, float32_t u);
inline void setPIsat(struct PI* pi, PI_SAT sat);
inline void setPIresetI(struct PI* pi);

// Zero differential delay order 1 CIC
struct CIC
{
    int32_t integrator;
    int32_t delay;
    float32_t out;
    uint16_t R; //Decimation rate
    uint16_t r; // Decimation counter
    float32_t inv_DCgain; //Filter Gain

};

void CIC_Init(struct CIC* CIC, uint16_t order);
void CIC_clear(struct CIC* CIC);
void CIC_Input(struct CIC* CIC, int32_t input);

struct Lp
{
    float32_t alpha;
    float32_t state;
};

void init_Lp(struct Lp* lp, float32_t fc, float32_t ts);
void calcLp(struct Lp* lp, float32_t u);

//
// Biquad filter
//

typedef struct Biquadcoefs{
    float32_t b[3];
    float32_t a[2];
    float32_t g;
}biquadCoefs;

typedef struct Biquad{
    biquadCoefs coefs;
    float32_t z[2];
    float32_t out;
}biquad;


void biquadReset(biquad* bq);

void biquadUpdateCoeffs(biquad* bq, const biquadCoefs ref);

float32_t biquadCalc(float32_t in, biquad*restrict bq);

void calcBiquad_HP(biquad* bq, float32_t Fc, float32_t Fs, float32_t Q);

void calcBiquad_Notch(biquadCoefs *coeff, float32_t Ts,
                                   float32_t notch_freq,
                                   float32_t c1, float32_t c2);

void calcBiquad_Resonant(biquadCoefs *coeff, float32_t Ts,
                                   float32_t w0,
                                   float32_t wc, float32_t ki);


//
// PLL
//

struct SPLL {
    float32_t  VAl; // Synchronous Voltage term
    float32_t  VBt; // Synchronous leading term
    float32_t  Amp; // Grid voltage amplitude estimate
    float32_t  sin; // PLL sine
    float32_t  cos; // PLL Cos
    float32_t  w; // PLL frequency
    float32_t  theta; // PLL Phase

    float32_t  Kia; // Amplitude integrator gain

    float32_t  wInteg; // Frequency integral error
    float32_t  Kiw; // Frequency integral gain
    float32_t  Kpw; //  Frequency propotional gain


    float32_t  w0; // initial frequency guess

    //Frequency limits
    float32_t wMin;
    float32_t wMax;

    float32_t lock; // flag for spll has matched the grid frequency
    uint32_t delayCount; // Fixed delay before locking

    float32_t sampleTime; // Sample time of the pll

    struct Lp wlp; //lowpass for the synchronous frequency of the pll, used to trigger the lock
    struct Lp alp; //Lowpass of the amplitude
    struct Lp erlp; //lowpass for the synchronous error of the pll, used to trigger the lock
};


void init_SPLL(struct SPLL* PLL, float32_t f0, float32_t fMin, float32_t fMax, float32_t Kiw, float32_t Kpw, float32_t Kia, float32_t a0, float32_t Ts);

inline float PLLcal_DC( struct SPLL *PLL, float32_t Vin);
inline float PLLcal( struct SPLL *restrict PLL, float32_t Vin);


//Test PLL and pwm
struct testSin{
    float32_t sin;
    float32_t a;
    float32_t f;
    float32_t phi;

    uint32_t t; // time index
};
extern struct testSin testsin;
void init_testSin(float32_t a, float32_t f, float32_t phi, float32_t ts);
inline float32_t nextSinVal();


#endif /* CONTROL_H_ */
