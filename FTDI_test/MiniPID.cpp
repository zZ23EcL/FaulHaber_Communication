//
// Created by zz23ecl on 22-7-15.
//

/**
* Small, easy to use PID implementation with advanced controller capability.<br>
* Minimal usage:<br>
* setPID(p,i,d); <br>
* ...looping code...{ <br>
* output=getOutput(sensorvalue,target); <br>
* }
*
* @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
*/

#include "MiniPID.h"

//**********************************
//Constructor functions
//**********************************
MiniPID::MiniPID(int p,int d){
    init();
    P=p; D=d;
}
MiniPID::MiniPID(int p, int d, int f){
    init();
    P=p; D=d; F=f;
}
void MiniPID::init(){
    P=0;
    D=0;
    F=0;

    maxError=0;
    errorSum=0;
    maxOutput=0;
    minOutput=0;
    setpoint=0;
    lastActual=0;
    firstRun=true;
    reversed=false;
    outputRampRate=0;
    lastOutput=0;
    outputFilter=0;
    setpointRange=0;
}

//**********************************
//Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this->is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this->is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
 */
void MiniPID::setP(int p){
    P=p;
    checkSigns();
}


void MiniPID::setD(int d){
    D=d;
    checkSigns();
}

/**Configure the FeedForward parameter. <br>
 * this->is excellent for Velocity, rate, and other	continuous control modes where you can
 * expect a rough output value based solely on the setpoint.<br>
 * Should not be used in "position" based control modes.
 *
 * @param f Feed forward gain. Affects output according to <b>output+=F*Setpoint</b>;
 */
void MiniPID::setF(int f){
    F=f;
    checkSigns();
}

/** Create a new PID object.
 * @param p Proportional gain. Large if large difference between setpoint and target.
 * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly.
 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
 */
void MiniPID::setPID(int p,  int d){
    P=p;D=d;
    checkSigns();
}

void MiniPID::setPID(int p,  int d,int f){
    P=p;D=d;F=f;
    checkSigns();
}



/**Specify a maximum output. If a single parameter is specified, the minimum is
 * set to (-maximum).
 * @param output
 */
void MiniPID::setOutputLimits(int output){ setOutputLimits(-output,output);}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(int minimum,int maximum){
    if(maximum<minimum)return;
    maxOutput=maximum;
    minOutput=minimum;
}

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void MiniPID::setDirection(bool reversed){
    this->reversed=reversed;
}

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void MiniPID::setSetpoint(int setpoint){
    this->setpoint=setpoint;
}

/** Calculate the PID value needed to hit the target setpoint.
* Automatically re-calculates the output at each call.
* @param actual The monitored value
* @param target The target value
* @return calculated output value for driving the actual to the target
*/
int MiniPID::getOutput(int actual, int setpoint){
    int output;
    int Poutput;
    int Doutput;
    int Foutput;

    this->setpoint=setpoint;

    //Ramp the setpoint used for calculations if user has opted to do so
    if(setpointRange!=0){
        setpoint=clamp(setpoint,actual-setpointRange,actual+setpointRange);
    }

    //Do the simple parts of the calculations
    int error=setpoint-actual;

    //Calculate F output. Notice, this->depends only on the setpoint, and not the error.
    Foutput=F*setpoint;

    //Calculate P term
    Poutput=P*error;

    //If this->is our first time running this-> we don't actually _have_ a previous input or output.
    //For sensor, sanely assume it was exactly where it is now.
    //For last output, we can assume it's the current time-independent outputs.
    if(firstRun){
        lastActual=actual;
        lastOutput=Poutput+Foutput;
        firstRun=false;
    }


    //Calculate D Term
    //Note, this->is negative. this->actually "slows" the system if it's doing
    //the correct thing, and small values helps prevent output spikes and overshoot

    Doutput= -D*(actual-lastActual);
    lastActual=actual;



    //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
    // 1. maxIoutput restricts the amount of output contributed by the Iterm.
    // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
    // 3. prevent windup by not increasing errorSum if output is output=maxOutput


    //And, finally, we can just add the terms up
    output=Foutput + Poutput + Doutput;

    //Figure out what we're doing with the error.
    if(minOutput!=maxOutput && !bounded(output, minOutput,maxOutput) ){
        errorSum=error;
        // reset the error sum to a sane level
        // Setting to current error ensures a smooth transition when the P term
        // decreases enough for the I term to start acting upon the controller
        // From that point the I term will build up as would be expected
    }
    else if(outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate) ){
        errorSum=error;
    }
    else{
        errorSum+=error;
    }

    //Restrict output to our specified output and ramp limits
    if(outputRampRate!=0){
        output=clamp(output, lastOutput-outputRampRate,lastOutput+outputRampRate);
    }
    if(minOutput!=maxOutput){
        output=clamp(output, minOutput,maxOutput);
    }
    if(outputFilter!=0){
        output=lastOutput*outputFilter+output*(1-outputFilter);
    }

    lastOutput=output;
    return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual valuess
 * @return calculated output value for driving the actual to the target
 */
int MiniPID::getOutput(){
    return getOutput(lastActual,setpoint);
}

/**
 *
 * @param actual
 * @return calculated output value for driving the actual to the target
 */
int MiniPID::getOutput(int actual){
    return getOutput(actual,setpoint);
}

/**
 * Resets the controller. this->erases the I term buildup, and removes D gain on the next loop.
 */
void MiniPID::reset(){
    firstRun=true;
    errorSum=0;
}

/**Set the maximum rate the output can increase per cycle.
 * @param rate
 */
void MiniPID::setOutputRampRate(int rate){
    outputRampRate=rate;
}

/** Set a limit on how far the setpoint can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
 * <br>this->limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void MiniPID::setSetpointRange(int range){
    setpointRange=range;
}

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
 * Uses an exponential rolling sum filter, according to a simple <br>
 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void MiniPID::setOutputFilter(int strength){
    if(strength==0 || bounded(strength,0,1)){
        outputFilter=strength;
    }
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise
 */
int MiniPID::clamp(int value, int min, int max){
    if(value > max){ return max;}
    if(value < min){ return min;}
    return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::bounded(int value, int min, int max){
    return (min<value) && (value<max);
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void MiniPID::checkSigns(){
    if(reversed){	//all values should be below zero
        if(P>0) P*=-1;
        if(D>0) D*=-1;
        if(F>0) F*=-1;
    }
    else{	//all values should be above zero
        if(P<0) P*=-1;
        if(D<0) D*=-1;
        if(F<0) F*=-1;
    }
}

