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

import org.tahomarobotics.robot.path.PathActions.PathAction;
import org.tahomarobotics.robot.state.Pose2D;
import org.tahomarobotics.robot.util.MathUtil;

/**
 * Path Builder enables the creation of a segmented path with the addition of lines and curve segments from 
 * a starting position and heading.
 *
 */
public class PathBuilder {

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

	private final Mirror mirror;
	private final PathDirection direction;

	// total length as path get built, used in creating path actions
	private double totalLength;

	private final List<PathSection> sections = new ArrayList<>();
	private Pose2D startPose;

	private final PathActions pathActions = new PathActions();

	/**
	 * Creates a path builder for constructing a path with lines and arcs starting at the specified
	 * starting location and automatically mirroring the path per the mirroring specification.
	 *
	 * @param direction - forward or reversed
	 * @param mirror - mirroring specification
	 * @param initialPose - starting position
	 */
	public PathBuilder(PathDirection direction, Mirror mirror, Pose2D initialPose) {
		this.direction = direction;
		this.mirror = mirror;

		startPose = mirrorPose2D(initialPose, mirror);
		if (direction == PathDirection.Reversed) {
			startPose.heading += 180;
			startPose.heading = MathUtil.normalizeAngleDegrees(startPose.heading);
		}
	}

	/**
	 * Return the list of sections making up this path.
	 *
	 * @return list of path sections
	 */
	public List<PathSection> getSections() {
		return sections;
	}

	/**
	 * Takes a way-point and mirrors it about the specified axis.
	 *
	 * @param waypoint - robot way-point
	 * @param mirror - mirroring specification
	 * @return mirrored robot way-point
	 */
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

	/**
	 * Takes a angle or heading and mirrors it about the specified axis.
	 *
	 * @param angle - robot heading
	 * @param mirror - mirroring specification
	 * @return mirrored robot heading
	 */
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

	public static double advMirrorAngle(double angle, Mirror mirror){
		double heading = angle;
		switch(mirror) {

			case X:
			case Y:
				heading = -heading;

			case None:
			case Both:
				// no change
		}
		if (mirror == Mirror.X || mirror == Mirror.Both) {
			heading = MathUtil.normalizeAngleDegrees(heading + 180);
		}
		return heading;
	}
	/**
	 * Takes a robot pose and mirrors it's location and heading about the specified axis.
	 *
	 * @param pose - robot pose
	 * @param mirror - mirroring specification
	 * @return mirrored robot pose
	 */
	public static Pose2D mirrorPose2D(Pose2D pose, Mirror mirror) {
		double heading = pose.heading;
		if (mirror == Mirror.X || mirror == Mirror.Both) {
			heading = MathUtil.normalizeAngleDegrees(heading + 180);
		}
		Waypoint pt = PathBuilder.mirrorPoint(new Waypoint(pose.x, pose.y, 0), mirror);
		return new Pose2D(pt.x, pt.y, heading);
	}

	/**
	 * Add a line path segment in the direction of the current heading for the specified length.
	 * MaxSpeed constrains the speed for velocity profiling later.  Path actions can be optionally added.
	 *
	 * @param length - length of the line path segment (inches)
	 * @param maxSpeed - maximum speed constrain (inches/second)
	 * @param actions - optional path action for path commanding
	 */
	public void addLine(double length, double maxSpeed, PathAction... actions) {

		PathSection section = new PathSection(length, maxSpeed, startPose);
		startPose = section.endPose;
		sections.add(section);

		pathActions.setupPathActions(totalLength, length, actions);
		totalLength += length;
	}

	/**
	 * Add a curved path segment starting in the direction of the heading and curving for the specified angle and radius.
	 * MaxSpeed constrains the speed for velocity profiling later.  Path actions can be optionally added.
	 *
	 * @param angle - curve arc angle (degrees), positive is for counter-clockwise
	 * @param radius - curve radius (inches), always positive
	 * @param maxSpeed - maximum speed constrain (inches/second)
	 * @param actions - optional path action for path commanding
	 */
	public void addArc(double angle, double radius, double maxSpeed, PathAction... actions) {

		PathSection section = new PathSection(mirrorAngle(angle, mirror), radius, maxSpeed, startPose);
		startPose = section.endPose;
		sections.add(section);

		pathActions.setupPathActions(totalLength, section.length, actions);
		totalLength += section.length;
	}

	/**
	 * Add a curved path segment starting in the direction of the path heading and curving to the specified end-point.  This
	 * automatically calculates the angle and radius to end at the specified point.  MaxSpeed constrains the speed for
	 * velocity profiling later.  Path actions can be optionally added.
	 *
	 * @param x - x location on the field (inches from origin)
	 * @param y - y location on the field (inches from origin)
	 * @param maxSpeed - maximum speed constrain (inches/second)
	 * @param actions - optional path action for path commanding
	 */
	public void addArcToPoint(double x, double y, double maxSpeed, PathAction... actions) {

		// calculate radius
		Waypoint point = new Waypoint(x, y, maxSpeed);
		point = mirrorPoint(point, mirror);
		double dist = point.distance(startPose.x, startPose.y);
		double angle = MathUtil.normalizeAngle(point.angle(startPose.x, startPose.y) - MathUtil.normalizeAngle(Math.toRadians(startPose.heading)));
		angle = mirrorAngle(angle, mirror);
		double radius = Math.abs((dist / 2) / Math.sin(angle));

		// create arc
		addArc(2 * Math.toDegrees(angle), radius, maxSpeed, actions);
	}

	/**
	 * Return the final pose of this path.
	 *
	 * @return pose of the path end-point
	 */
	public Pose2D getFinalPose() {
		Pose2D pose2D = new Pose2D(startPose);
		if (direction == PathDirection.Reversed) {
			pose2D.reverse();
		}
		return PathBuilder.mirrorPose2D(pose2D, mirror);
	}

	/**
	 * Return a description of the path segments.
	 */
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (PathSection section : sections) {

			sb.append(section).append('\n');
		}
		return sb.toString();
	}

	/**
	 * Return a generated list of way-points representing the path.
	 *
	 * @return list of path way-points
	 */
	public List<Waypoint> createWaypoints() {
		List<Waypoint> waypoints = new ArrayList<>();

		Pose2D end = null;
		for(PathSection section : sections) {


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

	/**
	 * Return the path actions which maintain each of the path added actions.
	 *
	 * @return path actions
	 */
	public PathActions getPathActions() {
		return pathActions;
	}

}
