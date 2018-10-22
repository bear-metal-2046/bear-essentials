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
package org.tahomarobotics.robot.state;

public class RobotSpeed {
	public double forward;
	public double rotational;

	public RobotSpeed() {
		this(0, 0);
	}

	public RobotSpeed(RobotSpeed other) {
		update(other.forward, other.rotational);
	}

	public RobotSpeed(double forward, double rotational) {
		update(forward, rotational);
	}

	public void reduce(double scale) {
		scale = Math.abs(scale);
		if (scale < 1.0) {
			forward *= scale;
			rotational *= scale;
		}
	}

	public void update(double forward, double rotational) {
		this.forward = forward;
		this.rotational = rotational;
	}
	
	public void copyFrom(RobotSpeed other) {
		update(other.forward, other.rotational);
	}

	@Override
	public String toString() {
		return String.format("Speed: %6.3f %6.3f", forward, rotational);
	}
}
