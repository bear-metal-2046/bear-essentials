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
package org.tahomarobotics.robot.path.Followers;

import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.path.CompletionListener;
import org.tahomarobotics.robot.path.Path;
import org.tahomarobotics.robot.path.PathController;
import org.tahomarobotics.robot.path.Waypoint;
import org.tahomarobotics.robot.state.Pose2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Adaptive Pure Pursuit Controller is proportional controller which follows a path or sequence of points.  The
 * gain of the controller is determined by the look ahead distance where the robot is command to drive to.
 * Using the current robot position, the closest point on the path to the robot is determined.  This value is
 * then used to calculate and cross track error and a new look ahead point placed the a distance in advance of 
 * the current location.  The distance is a constant with the error added to it.
 */
public class AdaptivePurePursuitController implements PathController {

	private static final Logger LOGGER = LoggerFactory.getLogger(AdaptivePurePursuitController.class);	
	
	private final Path path;
	private final double lookAheadDistance;
	private boolean complete = false;
	private double remainingDistance;
	private final double totalDistance;
	
	
	/**
	 * Constructs the path following controller.
	 * 
	 * @param waypoints - list of way-points that make up the path
	 * @param lookAheadDistance - look ahead distance used to tune the gain of the controller
	 */
	public AdaptivePurePursuitController(final List<Waypoint> waypoints, final double lookAheadDistance) {
		this.path = new Path(waypoints);
		this.lookAheadDistance = lookAheadDistance;
		
		path.start(new CompletionListener() {

			@Override
			public void onCompletion() {
				LOGGER.info("Finished path.");			
				complete = true;
			}
		});
				
		totalDistance = remainingDistance = path.getRemainingDistance();
	
		
	}
	
	/**
	 * Resets the path controller to start from the beginning of the path.
	 */
	public void reset() {
		complete = false;
		path.start(new CompletionListener() {

			@Override
			public void onCompletion() {
				LOGGER.info("Finished path.");			
				complete = true;
			}
		});
		
		remainingDistance = path.getRemainingDistance();
	}
	
	public boolean isComplete() {
		return complete;
	}
	
	public double getDistance() {
		return totalDistance - remainingDistance;
	}
	
	/**
	 * Using the current position, progress the current path segment and 
	 * calculate the cross track error.  Command the vehicle to join the
	 * path at the look ahead point (look ahead distance and errors added).
	 */
	@Override
	public double update(final Pose2D pose) {
				
		// update path segment with the current position 
		double pathError = path.update(pose);

		if (isComplete()) {
			return 0;
		}

		// find look ahead point on path
		Waypoint lookAheadPoint = path.getLookAheadPoint(lookAheadDistance + pathError);
	
		remainingDistance = path.getRemainingDistance();
		
		// create curve to join path at the look ahead point
		double curvature = getJoinCurvature(pose, lookAheadPoint);
		
		Waypoint closest = path.getClosestPoint();
		LOGGER.debug(String.format("Cmd: %7.3f Robot: %7.3f %7.3f %7.3f Path: %7.3f %7.3f Remaining: %7.3f Lookahead Point %7.3f %7.3f",
			curvature, pose.x, pose.y, pose.heading, 
			closest.x, closest.y, remainingDistance,
			lookAheadPoint.x, lookAheadPoint.y));
		
		SmartDashboard.putNumberArray("LookAhead", new double[] {pose.x, pose.y, pose.heading, curvature, lookAheadPoint.x, lookAheadPoint.y});
		
		return curvature;
	}

	/**
	 * Rejoin the path with a curve tangent to the current heading to
	 * the look ahead point.
	 * 
	 * @return curvature ( = 1/radius ) 
	 */
	private double getJoinCurvature(Pose2D pose, Waypoint lookAheadPoint) {

		double dx = lookAheadPoint.x - pose.x;
		double dy = lookAheadPoint.y - pose.y;
		double heading = Math.toRadians(pose.heading);
		double x = dy * Math.cos(heading) - dx * Math.sin(heading);
		double curvature = 2.0 * x / (dx * dx + dy * dy);

		return Double.isNaN(curvature) ? 0 : curvature;
	}
}
