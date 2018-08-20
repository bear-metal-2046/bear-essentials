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
package org.tahomarobotics.robot.path;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.motion.MotionController;
import org.tahomarobotics.robot.motion.MotionProfile;
import org.tahomarobotics.robot.motion.MotionState;
import org.tahomarobotics.robot.state.Pose2D;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public abstract class PathCommand extends Command {

	private static final Logger LOGGER = LoggerFactory.getLogger(PathCommand.class);
	
	public enum PathDirection {
		Forward(1), Reversed(-1);
		
		public final double sign;
		
		private PathDirection(double sign) {
			this.sign = sign;
		}
	}
	
	public enum Mirror {
		None, X, Y, Both
	}
	
	private static final double CURVE_SEG_SIZE = 5.0;
	
	private final List<Waypoint> waypoints = new ArrayList<>();
	protected final PathDirection direction;
	private final Mirror mirror;
	private final List<MotionProfile> motionProfiles;
	private final PathController pathController;
	private final MotionController motionController;
	
	private final Pose2D pose = new Pose2D();
	private final MotionState setpoint = new MotionState();
	private final MotionState current = new MotionState();
	
	private double pathDirection;
	private Waypoint lastPoint = null;
	private double pathStart;
	private int profileIndex;
	private boolean motionComplete;
	
	public PathCommand(final PathDirection direction, final Mirror mirror, final PathCommand priorPath) {
		this(direction, mirror, priorPath.getFinalPose());
	}
	
	public PathCommand(final PathDirection direction, final Mirror mirror, final Pose2D initialPose) {
		this.direction = direction;
		this.mirror = mirror;
		
		// setup initial position and direction
		addWaypoint(new Waypoint(initialPose.x, initialPose.y, 0.0));
		pathDirection = initialPose.heading;
		if (direction == PathDirection.Reversed) {
			pathDirection += 180;
		}
		
		// create the path
		createPath();
		
		// create the path direction controller
		pathController = createPathController(waypoints, direction);
		
		// create motion profiles
		motionProfiles = createMotionProfiles(waypoints);
		

		motionController = createMotionController();
	}
	
	protected abstract PathController createPathController(List<Waypoint> waypoints, PathDirection direction);
	
	protected abstract List<MotionProfile> createMotionProfiles(List<Waypoint> waypoints);
	
	protected abstract MotionController createMotionController();

	@Override
	protected void initialize() {
		pathStart = Timer.getFPGATimestamp();
		motionComplete = false;
		profileIndex = 0;
		motionController.reset();
	}

	@Override
	protected void execute() {
		double elapsedTime = Timer.getFPGATimestamp() - pathStart;
		
		getState(pose, current, direction);

		pose.heading = Math.toRadians(pose.heading);
		if (direction == PathDirection.Reversed) {
			pose.heading = PathCommand.normalizeAngle(pose.heading + Math.PI);
		}

		// steering command is the required curvature (1/radius of a circle)
		double steeringCommand = pathController.update(pose);
		
		// get setpoint from profiles
		if (!motionComplete) {
			if (!motionProfiles.get(profileIndex).getSetpoint(elapsedTime, setpoint)) {
				motionComplete = ++profileIndex >= motionProfiles.size();
			}
			if (LOGGER.isDebugEnabled()) {
				LOGGER.debug("setpoint: " + setpoint);
			}
		}
		
		// update motion controller
		double motionCommand = motionController.update(elapsedTime, current, setpoint);
		
		// apply power
		applyPower(motionCommand * direction.sign, steeringCommand * direction.sign, elapsedTime, current, setpoint);
		
	}

	protected abstract void getState(Pose2D pose, MotionState current, PathDirection direction);
	
	protected abstract void applyPower(double motionCommand, double steeringCommand, 
			double elapsedTime, MotionState current, MotionState setpoint);
	
	@Override
	protected boolean isFinished() {
		return motionComplete && (motionController.onTarget() || isTimedOut());
	}

	protected abstract void createPath();
	
	protected void addWaypoint(Waypoint waypoint) {
		lastPoint = waypoint;
		waypoints.add(mirrorPoint(waypoint, mirror));
	}
	
	public static Waypoint mirrorPoint(Waypoint waypoint, Mirror mirror) {
		Waypoint pt = new Waypoint(waypoint);
		switch(mirror) {
		
		case None:
			break;
			
		case X:
			pt.x = PathConstants.FIELD_LENGTH - pt.x;
			break;
			
		case Y:
			pt.y = PathConstants.FIELD_WIDTH  - pt.y;
			break;
			
		case Both:
			pt.x = PathConstants.FIELD_LENGTH - pt.x;
			pt.y = PathConstants.FIELD_WIDTH  - pt.y;
			break;
		}
		
		return pt;
	}
	
	public static Pose2D mirrorPose2D(Pose2D pose, Mirror mirror) {
		Waypoint pt = PathCommand.mirrorPoint(new Waypoint(pose.x, pose.y, 0), mirror);
		double heading = normalizeAngle(pose.heading + (mirror == Mirror.Y ? 180 : 0));
		return new Pose2D(pt.x, pt.y, heading);
	}


	protected void addLine(double length, double maxSpeed) {

		Waypoint waypoint = new Waypoint(lastPoint);
		double pathDirRadians = Math.toRadians(pathDirection);
		waypoint.x += length * Math.cos(pathDirRadians);
		waypoint.y += length * Math.sin(pathDirRadians);
		waypoint.speed = maxSpeed;
		addWaypoint(waypoint);
	}
	
	protected void addArc(double angle, double radius, double maxSpeed) {
				
		// convert arc to line segments
		int numSegments = (int)(Math.abs(angle)/CURVE_SEG_SIZE) + 1;
		double deltaAngle = angle / numSegments;
		double endPointLen = Math.abs(Math.toRadians(deltaAngle)) * radius;
		
		for (int i = 0; i < numSegments; i++) {
			
			double endPointAngle = Math.toRadians(pathDirection + deltaAngle/2);

			addWaypoint(new Waypoint(
					lastPoint.x + endPointLen * Math.cos(endPointAngle), 
					lastPoint.y + endPointLen * Math.sin(endPointAngle),
					maxSpeed));
			
			pathDirection += deltaAngle;
		}
	}

	protected void addArcToPoint(double x, double y, double maxSpeed) {
		// reflect point and angle
		Waypoint point = new Waypoint(x, y, maxSpeed);
		
		// calculate radius
		double dist = point.distance(lastPoint);
		double angle = normalizeAngle(point.angle(lastPoint) - normalizeAngle(Math.toRadians(pathDirection)));
		double radius = Math.abs((dist / 2) / Math.sin(angle));
		
		// create arc
		addArc(2 * Math.toDegrees(angle), radius, maxSpeed);
	}
	
	private static double normalizeAngle(double angle) {
		double newAngle = angle;
	    while (newAngle <= -Math.PI) newAngle += Math.PI * 2;
	    while (newAngle > Math.PI) newAngle -= Math.PI * 2;
	    return newAngle;
	}
	
	public Pose2D getFinalPose() {
		Pose2D pose2D = new Pose2D(lastPoint.x, lastPoint.y, pathDirection);
		if (direction == PathDirection.Reversed) {
			pose2D.reverse();
		}
		return pose2D;
	}

	public List<Waypoint> getWaypoints() {
		return waypoints;
	}
}
