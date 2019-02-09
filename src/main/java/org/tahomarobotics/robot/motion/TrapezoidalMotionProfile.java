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

public class TrapezoidalMotionProfile extends MotionProfile {

	public TrapezoidalMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration) throws MotionProfileException {
		super(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration);
	}

	@Override
	protected MotionState[] generatePhases() throws MotionProfileException {
		
		final double distance = endPosition - startPosition;
		final double abs_distance = Math.abs(distance);
		final double direction = Math.signum(distance);
		
		double max_velocity = Math.min(
				maxVelocity,
				Math.sqrt(abs_distance*maxAcceleration + startVelocity * startVelocity / 2 
						+ endVelocity * endVelocity / 2));


		final double ta = Math.max(0, (max_velocity - direction * startVelocity) / maxAcceleration);
		final double td = Math.max(0, (max_velocity - direction * endVelocity) / maxAcceleration);
		double tv = abs_distance > 0 ? ((abs_distance - 0.5*ta*(max_velocity + direction * startVelocity) - 0.5*td*(max_velocity + direction * endVelocity)) / max_velocity) : 0;
		if (tv < 0 && tv > -0.0001) {
			tv = 0;
		}
	
		if (ta < 0 || td < 0 || tv < 0) {
			
			throw new MotionProfileException(String.format("Failed to resolve constraint while creating Trapezoidal profile ta=%f td=%f tv=%f \n%s", ta, td, tv, this));
		}
		
		double max_acceleration = direction * maxAcceleration;
		max_velocity *= direction;
		
		MotionState phases[] = new MotionState[4];

		// initial state
		MotionState initial = phases[0] = new MotionState()
				.setTime(startTime)
				.setPosition(startPosition)
				.setVelocity(startVelocity)
				.setAcceleration(max_acceleration);
		
		// end of constant acceleration
		initial = phases[1] = getPhaseSetpoint(ta, initial, new MotionState())
				.setVelocity(max_velocity)
				.setAcceleration(0);
		
		// end of constant velocity
		initial = phases[2] = getPhaseSetpoint(tv, initial, new MotionState())
				.setAcceleration(-max_acceleration);

		// end of constant deceleration
		initial = phases[3] = getPhaseSetpoint(td, initial, new MotionState())
				.setVelocity(endVelocity)
				.setPosition(endPosition)
				.setAcceleration(0);

		return phases;
	}
}
