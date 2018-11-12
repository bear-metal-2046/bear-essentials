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

import java.util.List;

public class MotionProfiles {
	
	private final List<MotionProfile> fwdMotionProfiles;
	private final List<MotionProfile> rotMotionProfiles;
	
	private int index = 0;
	
	private final double totalDuration;
	
	private final MotionState lastForwardSetpoint;
	private final MotionState lastRotationSetpoint;
	
	public MotionProfiles(final List<MotionProfile> motionProfiles) {
		this(motionProfiles, null);
	}
	
	public MotionProfiles(final List<MotionProfile> fwdMotionProfiles, final List<MotionProfile> rotMotionProfiles) {
		this.fwdMotionProfiles = fwdMotionProfiles;
		this.rotMotionProfiles = rotMotionProfiles;
		
		MotionProfile lastProfile = fwdMotionProfiles.get(fwdMotionProfiles.size() - 1);
		totalDuration = lastProfile.getEndTime();
		lastForwardSetpoint = lastProfile.getLastMotionState();
		
		if(rotMotionProfiles != null) {
			lastProfile = rotMotionProfiles.get(rotMotionProfiles.size() - 1);
			lastRotationSetpoint = lastProfile.getLastMotionState();
		} else {
			lastRotationSetpoint = null;
		}
	}
	
	public void reset() {
		index = 0;
	}
	
	public boolean getSetpoint(final double elapsedTime, final MotionState setpoint) {
		return getSetpoint(elapsedTime, setpoint, null);
	}
	
	public boolean getSetpoint(final double elapsedTime, final MotionState setpoint, final MotionState rotSetpoint) {
		
		while(elapsedTime < totalDuration) {
			
			// return new set-point from current motion profile
			if (fwdMotionProfiles.get(index).getSetpoint(elapsedTime, setpoint)) {
				if (rotSetpoint != null) {
					rotMotionProfiles.get(index).getSetpoint(elapsedTime, rotSetpoint);
				}
				return false;
			}
			
			// cycle to the next motion profiles unless at the end of all profiles
			if (++index >= fwdMotionProfiles.size()) {
				break;
			}
		}
		
		setpoint.copy(lastForwardSetpoint);
		if (rotSetpoint != null) {
			rotSetpoint.copy(lastRotationSetpoint);	
		}
		return true;
	}
	
	public double getTotalDuration() {
		return totalDuration;
	}
}
