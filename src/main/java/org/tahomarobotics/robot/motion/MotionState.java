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

public class MotionState {

    public double time;
    public double position;
    public double velocity;
    public double acceleration;
    public double jerk;

    public void copy(MotionState other) {
    	this.time = other.time;
    	this.position = other.position;
    	this.velocity = other.velocity;
    	this.acceleration = other.acceleration;
    	this.jerk = other.jerk;
    }
    
    public MotionState setJerk(double jerk) {
    	this.jerk = jerk;
    	return this;
    }
    
    public MotionState setAcceleration(double acceleration) {
    	this.acceleration = acceleration;
    	return this;
    }
    
    public MotionState setVelocity(double velocity) {
    	this.velocity = velocity;
    	return this;
    }
    
    public MotionState setPosition(double position) {
    	this.position = position;
    	return this;
    }

    public MotionState setTime(double time) {
    	this.time = time;
    	return this;
    }
    @Override
	public String toString() {
    	return String.format("%6.1f,%6.1f,%6.1f", acceleration, velocity, position);
	}
    
    
}
