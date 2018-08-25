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

import java.util.List;

import org.tahomarobotics.robot.path.PathCommand.Mirror;
import org.tahomarobotics.robot.path.PathCommand.PathDirection;
import org.tahomarobotics.robot.state.Pose2D;

public class PathBuilder {

	private static final double CURVE_SEG_SIZE = 5.0;

	private final List<Waypoint> waypoints;
	private double pathDirection;
	private Waypoint lastPoint = null;
	private final Mirror mirror;
	protected final PathDirection direction;
		
	
	public PathBuilder(PathDirection direction, Mirror mirror, Pose2D initialPose, List<Waypoint> waypoints) {
		this.direction = direction;
		this.mirror = mirror;
		this.waypoints = waypoints;
		pathDirection = direction == PathDirection.Reversed ? initialPose.heading + 180 : initialPose.heading;

		// setup initial position and direction
		addWaypoint(new Waypoint(initialPose.x, initialPose.y, 0.0));
	}
	
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
		Waypoint pt = PathBuilder.mirrorPoint(new Waypoint(pose.x, pose.y, 0), mirror);
		double heading = normalizeAngle(pose.heading + (mirror == Mirror.Y ? 180 : 0));
		return new Pose2D(pt.x, pt.y, heading);
	}


	public void addLine(double length, double maxSpeed) {

		Waypoint waypoint = new Waypoint(lastPoint);
		double pathDirRadians = Math.toRadians(pathDirection);
		waypoint.x += length * Math.cos(pathDirRadians);
		waypoint.y += length * Math.sin(pathDirRadians);
		waypoint.speed = maxSpeed;
		addWaypoint(waypoint);
	}
	
	public void addArc(double angle, double radius, double maxSpeed) {
				
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

	public void addArcToPoint(double x, double y, double maxSpeed) {
		// reflect point and angle
		Waypoint point = new Waypoint(x, y, maxSpeed);
		
		// calculate radius
		double dist = point.distance(lastPoint);
		double angle = normalizeAngle(point.angle(lastPoint) - normalizeAngle(Math.toRadians(pathDirection)));
		double radius = Math.abs((dist / 2) / Math.sin(angle));
		
		// create arc
		addArc(2 * Math.toDegrees(angle), radius, maxSpeed);
	}
	
	public static double normalizeAngle(double angle) {
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

}
