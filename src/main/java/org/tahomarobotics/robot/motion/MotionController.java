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

	public MotionController(final double kP, final double kV, final double kI, final double kffV, final double kffA, final double positionTolerance) {
			this.kP = kP;
			this.kV = kV;
			this.kI = kI;
			this.kffV = kffV;
			this.kffA = kffA;
			this.positionTolerance = positionTolerance;
	}
	
	private double totalError;
	private double prevTime;
	private volatile boolean onTarget = true;
	
	public void reset() {
		prevTime = Double.NaN;
		totalError = 0;
		onTarget = false;
	}
	
	
	public double update(final double t, final MotionState currentState, final MotionState setpoint) {
		 		
		// Update error.
        double positionError = setpoint.position - currentState.position;
        double velocityError = setpoint.velocity - currentState.velocity;
    	totalError = Double.isNaN(prevTime) ? 0.0 : (totalError + positionError * (t - prevTime));        	
    	prevTime = t;
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
	
	public boolean onTarget() {
		return onTarget;
	}
}
