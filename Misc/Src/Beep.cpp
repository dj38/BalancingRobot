#include "Beep.h"

/** Create a Beep object connected to the specified PwmOut pin
 *
 * @param pin PwmOut pin to connect to
 */
//Beep::Beep(PinName pin) : m_pwm(pin)
Beep::Beep()
{
    //m_pwm.write(0.0);     // after creating it have to be off
    m_beepMode = defaultBeepMode;
}

/** Stop the beep instantaneously.
 */
void Beep::nobeep()
{
    //m_pwm.write(0.0);
}

/** Beep with given frequency and duration.
 *
 * @param frequency - the frequency of the tone in Hz
 * @param time - the duration of the tone in seconds, 0 runs indefinitely
 */
void Beep::beep(float freq, float time)
{
    switch (m_beepMode) {
        case std: {
            beepsDuration=time;
            break;
        }
        case lowNoise: {
            beepsDuration=0.0025;
            break;
        }
        case noNoise: {
            beepsDuration=0.000001;
            break;
        }
    }

    //m_pwm.period(1.0f/freq);
    //m_pwm.write(0.5);            // 50% duty cycle - beep on
    /*if (time > 0)
        toff.attach(this,&Beep::nobeep, beepsDuration);   // time to off
*/
}

void Beep::nobeep_()
{
    //m_pwm.write(0.0);
    //toff.attach(this,&Beep::beep_,(beepsPeriod-beepsDuration));   // time to on
    m_remainingBeeps--;
}

void Beep::beep_()
{
    //m_pwm.period(1.0f/beepsFreq);
    if (m_remainingBeeps < 1) return;
    //m_pwm.write(0.5);            // 50% duty cycle - beep on
    /*if (time > 0)
        toff.attach(this,&Beep::nobeep_, beepsDuration);   // time to off
        */
}

void Beep::setBeepMode(m_beepModet beepMode)
{
    m_beepMode = beepMode;
}


void Beep::repeatedBeep(float freq, float duration,float period,int nbBeeps)
{
    m_remainingBeeps=nbBeeps;
    switch (m_beepMode) {
        case std: {
            beepsDuration=duration;
            break;
        }
        case lowNoise: {
            beepsDuration=0.0025;
            break;
        }
        case noNoise: {
            beepsDuration=0.000001;
            break;
        }
    }
    beepsPeriod=period;
    beepsFreq=freq;
    beep_();
}
