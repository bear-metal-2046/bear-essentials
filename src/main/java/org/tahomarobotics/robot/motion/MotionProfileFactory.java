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

import java.util.ArrayList;
import java.util.List;

import org.tahomarobotics.robot.motion.MotionProfile.MotionProfileException;
import org.tahomarobotics.robot.path.Waypoint;

public class MotionProfileFactory {
	
	public enum Profile {
		Trapezoid, SCurve;
	}

	private static class MotionSection {
		public final double length;
		public final double maxVelocity;
		
		public MotionSection(double length, double maxVelocity) {
			this.length = length;
			this.maxVelocity = maxVelocity;
		}

		@Override
		public String toString() {
			return String.format("%5.1f %3.0f", length, maxVelocity);
		}
	}

	private static List<MotionSection> createMotionSections(List<Waypoint> waypoints) {
		List<MotionSection> sections = new ArrayList<>();
	
		double len = 0;
		double max = waypoints.get(1).speed;		
		Waypoint prev = waypoints.get(0);
		
		for (int i = 1; i < waypoints.size(); i++) {
			Waypoint next = waypoints.get(i);
			MotionSection section = new MotionSection(prev.distance(next), next.speed);
			
			if (max != section.maxVelocity) {
				sections.add(new MotionSection(len, max));	
				len = 0;
			}
			max = section.maxVelocity;
			len += section.length;
			prev = next;
		}
		sections.add(new MotionSection(len, max));
		
		return sections;
	}
	
	public static List<MotionProfile> createMotionProfiles(List<Waypoint> waypoints, Profile profile, double maxAccel, double maxJerk) {
		
		List<MotionSection> sections = createMotionSections(waypoints);
		
		List<MotionProfile> profiles = new ArrayList<>();
		
		double startTime = 0;
		double startVelocity = 0;
		double startPosition = 0;
		
		for(int i = 0; i < sections.size(); i++) {
			MotionSection section = sections.get(i);
			
			double endPosition = startPosition + section.length;
			double maxVelocity = section.maxVelocity;
			double nextVelocity = (i+1) < sections.size() ? sections.get(i+1).maxVelocity : 0; 
			double endVelocity = Math.min(maxVelocity, nextVelocity);
			
			MotionProfile motionProfile = createMotionProfile(profile, startTime, 
					startPosition, endPosition, 
					startVelocity, endVelocity,  
					maxVelocity, maxAccel, maxJerk);
			
			profiles.add(motionProfile);
			
			startTime = motionProfile.getEndTime();
			startVelocity = endVelocity;
			startPosition = endPosition;
		}		
		
		return profiles;
	}
	
	public static MotionProfile createMotionProfile(Profile profile, 
			double startTime, double startPosition, double endPosition, 
			double startVelocity, double endVelocity, double maxVelocity, 
			double maxAccel, double maxJerk) {
		try {
			switch(profile) {
			
			case Trapezoid:
				return new TrapezoidalMotionProfile(startTime, startPosition, endPosition, 
						startVelocity, endVelocity, maxVelocity, maxAccel);
			
			case SCurve:
				return new SCurveMotionProfile(startTime, startPosition, endPosition,
						startVelocity, endVelocity, maxVelocity, maxAccel, maxJerk);
			}
		} catch (MotionProfileException e) {
			e.printStackTrace();
		}
		return null;
	}
}
