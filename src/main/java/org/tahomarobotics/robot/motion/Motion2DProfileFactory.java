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

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.tahomarobotics.robot.motion.MotionProfile.MotionProfileException;
import org.tahomarobotics.robot.path.PathSection;
import org.tahomarobotics.robot.state.Pose2D;

public class Motion2DProfileFactory {

	private enum Profile {
		Trapezoid, SCurve;
	}

	/**
	 * Create a trapezoidal set of profiles for the provided Path constrained to the given acceleration.
	 *
	 * @param sections - path sections with distances and max velocity constraints
	 * @param maxAccel - max acceleration constraint
	 * @return MotionProfiles for retrieving set-points
	 */
	public static MotionProfiles createTrapezoidMotionProfile(List<PathSection> sections, double maxAccel) {
		List<MotionProfile> motionProfiles = new ArrayList<>();
		createMotionProfiles(sections, Profile.Trapezoid, maxAccel, 0, motionProfiles);
		return new MotionProfiles(motionProfiles);
	}

	/**
	 * Create a trapezoidal set of profiles for the provided Path constrained to the given acceleration. This also
	 * provides profiles for the rotation of the robot synchronized with the forward motion.
	 *
	 * @param sections - path sections with distances and max velocity constraints
	 * @param maxAccel - max acceleration constraint
	 * @param maxRotationalAccel - max angular acceleration constraint
	 * @return MotionProfiles for retrieving set-points
	 */
	public static MotionProfiles createTrapezoidMotionProfile(List<PathSection> sections, double maxAccel, double maxRotationalAccel) {

		List<MotionProfile> fwdMotionProfiles = new ArrayList<>();
		createMotionProfiles(sections, Profile.Trapezoid, maxAccel, 0, fwdMotionProfiles);

		List<MotionProfile> rotMotionProfiles = new ArrayList<>();
		createRotationalMotionProfiles(sections, Profile.Trapezoid, fwdMotionProfiles, maxRotationalAccel, 0, rotMotionProfiles);

		return new MotionProfiles(fwdMotionProfiles, rotMotionProfiles);
	}

	/**
	 * Create a trapezoidal set of profiles for the provided Path constrained to the given acceleration. This also
	 * provides profiles for the rotation of the robot synchronized with the forward motion.
	 *
	 * @param sections - path sections with distances and max velocity constraints
	 * @param maxAccel - max acceleration constraint
	 * @return MotionProfiles for retrieving set-points
	 */
	public static MotionProfiles createSwerveTrapezoidMotionProfile(List<PathSection> sections, double maxAccel) {

		List<PathSection> fwdSections = new ArrayList<>();

		List<PathSection> strSections = new ArrayList<>();

		for(PathSection vect : sections){
			fwdSections.add(vect.mult(Math.cos(Math.atan2((vect.endPose.y - vect.startPose.y),(vect.endPose.x - vect.startPose.x)))));
			strSections.add(vect.mult(Math.sin(Math.atan2((vect.endPose.y - vect.startPose.y),(vect.endPose.x - vect.startPose.x)))));
		}

		//fwd profiles
		List<MotionProfile> fwdMotionProfiles = new ArrayList<>();
		createMotionProfiles(fwdSections, Profile.Trapezoid, maxAccel, 0, fwdMotionProfiles);

		//str profiles
		List<MotionProfile> strMotionProfiles = new ArrayList<>();
		createMotionProfiles(strSections, Profile.Trapezoid, maxAccel, 0, strMotionProfiles);

		return new MotionProfiles(fwdMotionProfiles, strMotionProfiles);
	}

	/**
	 * Create a set of motion profiles from the provided path sections.  
	 *
	 * @param sections - path sections used to create a motion profile
	 * @param profile - Trapezoid and S-Curve
	 * @param maxAccel - acceleration to be used to change velocity
	 * @param maxJerk - jerk to be used to change acceleration (not used in Trapezoid)
	 * @param motionProfiles - resultant profiles
	 */
	private static void createMotionProfiles(List<PathSection> sections, Profile profile,
			double maxAccel, double maxJerk, List<MotionProfile> motionProfiles) {

		double startTime = 0;
		double startVelocity = 0;
		double startPosition = 0;

		for(int i = 0; i < sections.size(); i++) {

			PathSection section = sections.get(i);

			double endPosition = startPosition + section.length;
			double maxVelocity = section.maxVelocity;
			double nextVelocity = (i+1) < sections.size() ? sections.get(i+1).maxVelocity : 0;
			double endVelocity = Math.min(maxVelocity, nextVelocity);

			MotionProfile motionProfile = createMotionProfile(profile, startTime,
					startPosition, endPosition,
					startVelocity, endVelocity,
					maxVelocity, maxAccel, maxJerk);

			motionProfiles.add(motionProfile);

			startVelocity = endVelocity;
			startPosition = endPosition;
			startTime = motionProfile.getEndTime();
		}
	}


	private static void createRotationalMotionProfiles(List<PathSection> sections, Profile profile, List<MotionProfile> fwdProfiles,
			double maxRotAccel, double maxRotJerk, List<MotionProfile> rotProfiles) {

		double startTime = 0;

		for(int i = 0; i < sections.size(); i++) {
			PathSection section = sections.get(i);
			MotionProfile fwdMotionProfile = fwdProfiles.get(i);

			// time to turn arc
			double duration = fwdMotionProfile.getEndTime() - startTime;

			// minimum acceleration is a triangular motion
			double minAcceleration = 4*Math.abs(section.angle)/duration/duration;
			if (maxRotAccel < minAcceleration) {
				System.err.format("Rotational Acceleration too low %f < %f\n", maxRotAccel, minAcceleration);
				maxRotAccel = minAcceleration;
			}

			// trapezoidal rotational velocity completing on time
			section.maxRotationalVelocity = maxRotAccel/2*(duration-Math.sqrt(duration*duration-4*Math.abs(section.angle)/maxRotAccel));

			startTime = fwdMotionProfile.getEndTime();
		}

		startTime = 0;
		double startRotationalVelocity = 0;
		double startRotationalPosition = sections.get(0).startPose.heading;

		for(int i = 0; i < sections.size(); i++) {

			PathSection section = sections.get(i);

			double maxRotationalVelocity = section.maxRotationalVelocity;
			double direction = Math.signum(section.angle);
			double endRotationalPosition = startRotationalPosition + section.angle;
			double nextRotationalVelocity = (i+1) < sections.size() ? sections.get(i+1).maxRotationalVelocity : 0;
			double endRotationalVelocity = direction * Math.min(Math.abs(maxRotationalVelocity), Math.abs(nextRotationalVelocity));

			MotionProfile rotMotionProfile = createMotionProfile(profile, startTime,
					startRotationalPosition, endRotationalPosition,
					startRotationalVelocity, endRotationalVelocity,
					maxRotationalVelocity, maxRotAccel, maxRotJerk);

			rotProfiles.add(rotMotionProfile);

			startRotationalVelocity = endRotationalVelocity;
			startRotationalPosition = endRotationalPosition;

			startTime = fwdProfiles.get(i).getEndTime();
		}

	}

	private static MotionProfile createMotionProfile(Profile profile,
			double startTime, double startPosition, double endPosition,
			double startVelocity, double endVelocity, double maxVelocity,
			double maxAccel, double maxJerk) {
		try {
			switch(profile) {

			case Trapezoid:
				return new TrapezoidalMotionProfile(startTime, startPosition, endPosition,
						startVelocity, endVelocity, maxVelocity, maxAccel);

			case SCurve:

			}
		} catch (MotionProfileException e) {
			e.printStackTrace();
		}
		return null;
	}
}
