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

import org.tahomarobotics.robot.path.PathCommand.Mirror;
import org.tahomarobotics.robot.path.PathCommand.PathDirection;
import org.tahomarobotics.robot.state.Pose2D;
import org.tahomarobotics.robot.util.MathUtil;

public class PathBuilder {

	private static final double CURVE_SEG_SIZE = 5.0;

	private final List<Waypoint> waypoints = new ArrayList<>();
	private double pathDirection;
	private Waypoint lastPoint = null;
	private final Mirror mirror;
	protected final PathDirection direction;
	
	public class Section {
		
		public final double length;
		public final double angle;
		public final double radius;
		public final double maxVelocity;
		public double maxRotationalVelocity;
		public final Pose2D startPose;
		public final Pose2D endPose;
		
		
		public Section(double length, double maxVelocity, Pose2D startPose) {
			this.length = length;
			this.angle = 0.0;
			this.radius = 0.0;
			this.maxVelocity = maxVelocity;
			this.startPose = startPose;
			this.endPose =  new Pose2D(startPose);		
			
			double angleRadians = Math.toRadians(startPose.heading);
			endPose.x += length * Math.cos(angleRadians);
			endPose.y += length * Math.sin(angleRadians);
		}
		
		public Section(double angle, double radius, double maxVelocity, Pose2D startPose) {
			double angleRadians = Math.toRadians(angle);
			this.length = Math.abs(angleRadians) * radius;
			this.angle = angle;
			this.radius = radius;
			this.maxVelocity = maxVelocity;
			this.startPose = startPose;
			this.endPose =  new Pose2D(startPose);		

			double chord = 2.0 * radius * Math.sin(Math.abs(angleRadians)/2.0);
			double halfAngle = Math.toRadians(startPose.heading) + angleRadians / 2.0;
			
			endPose.heading += angle;
			endPose.x += chord * Math.cos(halfAngle);
			endPose.y += chord * Math.sin(halfAngle);
		}		
		
		@Override
		public String toString() {
			return String.format("Section: %6.1f %6.1f %6.1f - start%s - end%s", length, angle, maxVelocity, startPose, endPose);
		}
	}
	
	private final List<Section> sections = new ArrayList<>();
	private Pose2D startPose;
	
	public PathBuilder(PathDirection direction, Mirror mirror, Pose2D initialPose) {
		this.direction = direction;
		this.mirror = mirror;
	
		startPose = mirrorPose2D(initialPose, mirror);
		if (direction == PathDirection.Reversed) {
			startPose.heading -= 180;
		}
		pathDirection = direction == PathDirection.Reversed ? initialPose.heading + 180 : initialPose.heading;

		// setup initial position and direction
		addWaypoint(new Waypoint(initialPose.x, initialPose.y, 0.0));
	}
	
	public List<Section> getSections() {
		return sections;
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
	
	public static double mirrorAngle(double angle, Mirror mirror) {
		
		switch(mirror) {
		
		case X:
		case Y:
			angle = -angle;
		
		case None:
		case Both:
			// no change
		}
		return angle;
	}
	
	public static Pose2D mirrorPose2D(Pose2D pose, Mirror mirror) {
		double heading = MathUtil.normalizeAngleDegrees(pose.heading + (mirror == Mirror.X ? 180 : 0));
		Waypoint pt = PathBuilder.mirrorPoint(new Waypoint(pose.x, pose.y, 0), mirror);
		return new Pose2D(pt.x, pt.y, heading);
	}


	public void addLine(double length, double maxSpeed) {

		Section section = new Section(length, maxSpeed, startPose);
		startPose = section.endPose;
		sections.add(section);
		
		Waypoint waypoint = new Waypoint(lastPoint);
		double pathDirRadians = Math.toRadians(pathDirection);
		waypoint.x += length * Math.cos(pathDirRadians);
		waypoint.y += length * Math.sin(pathDirRadians);
		waypoint.speed = maxSpeed;
		addWaypoint(waypoint);
	}
	
	public void addArc(double angle, double radius, double maxSpeed) {
		
		Section section = new Section(mirrorAngle(angle, mirror), radius, maxSpeed, startPose);
		startPose = section.endPose;
		sections.add(section);

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
		double angle = MathUtil.normalizeAngle(point.angle(lastPoint) - MathUtil.normalizeAngle(Math.toRadians(pathDirection)));
		double radius = Math.abs((dist / 2) / Math.sin(angle));
		
		// create arc
		addArc(2 * Math.toDegrees(angle), radius, maxSpeed);
	}
	
	public Pose2D getFinalPose() {
		Pose2D pose2D = new Pose2D(lastPoint.x, lastPoint.y, pathDirection);
		if (direction == PathDirection.Reversed) {
			pose2D.reverse();
		}
		return pose2D;
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (Section section : sections) {
			
			sb.append(section).append('\n');
		}
		return sb.toString();
	}
	
	public List<Waypoint> createWaypoints() {
		List<Waypoint> waypoints = new ArrayList<>();
		
		Pose2D end = null;
		for(Section section : sections) {
			
			
			if (section.angle == 0.0) {
				waypoints.add(new Waypoint(section.startPose.x, section.startPose.y, section.maxVelocity));
			} else {
				
				int numSegments = (int)(Math.abs(section.angle)/CURVE_SEG_SIZE) + 1;
				double deltaAngle = section.angle / numSegments;
				double chord = 2.0 * section.radius * Math.sin(Math.toRadians(Math.abs(section.angle))/2.0/numSegments);
				
				Pose2D pose = new Pose2D(section.startPose);				
				for (int i = 0; i < numSegments; i++) {
					waypoints.add(new Waypoint(pose.x, pose.y, section.maxVelocity));
					double halfAngle = Math.toRadians(pose.heading + deltaAngle/2);
					pose.x += chord * Math.cos(halfAngle);
					pose.y += chord * Math.sin(halfAngle);
					pose.heading += deltaAngle;
				}
			}
			
			end = section.endPose;
		}
		waypoints.add(new Waypoint(end.x, end.y, 0.0));
		
		return waypoints;
	}


}
