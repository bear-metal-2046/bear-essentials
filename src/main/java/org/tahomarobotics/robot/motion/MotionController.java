/**
 * Copyright 2018 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files (the "Software"), to deal in the Software without restriction, including without 
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following 
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions 
 * of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 * 
 */
package org.tahomarobotics.robot.motion;

public class MotionController {

	public final double kP;
	public final double kV;
	public final double kI;
	public final double kffV;
	public final double kffA;
	public final double positionTolerance;

	private double totalError;
	private double prevTime;
	private volatile boolean onTarget = true;
	private double positionError;
	private double velocityError;

	/**
	 * Motion Controller
	 * 
	 * @param kP - positional feed-back gain
	 * @param kV - velocity feed-back gain
	 * @param kI - integration feed-back gain
	 * @param kffV - velocity feed=forward gain
	 * @param kffA - acceleration feed-forward gain
	 * @param positionTolerance - positional tolerance
	 */
	public MotionController(final double kP, final double kV, final double kI, final double kffV, final double kffA, final double positionTolerance) {
			this.kP = kP;
			this.kV = kV;
			this.kI = kI;
			this.kffV = kffV;
			this.kffA = kffA;
			this.positionTolerance = positionTolerance;
	}
	

	/**
	 * Clears out any previously held data
	 */
	public void reset() {
		prevTime = Double.NaN;
		totalError = 0;
		onTarget = false;
	}
	
	/**
	 * Controller update which calculates the controller output based on the current state and the provided set-point.
	 * This calculates the output based on feed-forward and feed-back gains.
	 * 
	 * @param time - elapsed time
	 * @param currentState - current motion state for position and velocity
	 * @param setpoint - set-point for position, velocity and acceleration
	 * @return calculated controller output
	 */
	public double update(final double time, final MotionState currentState, final MotionState setpoint) {
		if(Math.abs(Double.isNaN(time) ? 0.0 : time - prevTime) > 0.001){
			reset();
		}
		 		
		// Update error.
        positionError = setpoint.position - currentState.position;
        velocityError = setpoint.velocity - currentState.velocity;
    	totalError = Double.isNaN(prevTime) ? 0.0 : (totalError + positionError * (time - prevTime));        	
    	prevTime = time;
        // Calculate the feed forward and proportional terms.
        double output = 
        		kffV * setpoint.velocity + 
        		kffA * setpoint.acceleration + 
        		kP * positionError + 
        		kV * velocityError + 
        		kI * totalError;
        
        
        
        onTarget = Math.abs(positionError) <= positionTolerance;

        return output;
	}
	
	public double getPositionError() {
		return positionError;
	}
	
	/**
	 * Indicates if the current state is within positional tolerance of the set-point
	 * @return - true if within positional tolerance
	 */
	public boolean onTarget() {
		return onTarget;
	}


	@Override
	public String toString() {
		return String.format("%7.2f,%7.2f,%7.2f", positionError, velocityError, totalError);
	}
	
	
}
