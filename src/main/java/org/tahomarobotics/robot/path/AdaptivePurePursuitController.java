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
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.tahomarobotics.robot.state.Pose2D;


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
	 * @param path
	 * @param lookAheadDistance - look ahead distance used to tune the gain of the controller
	 * @param reversed - indicates if the robot needs to follow the path going in reverse, otherwise forward
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
		double x = dy * Math.cos(pose.heading) - dx * Math.sin(pose.heading);
		double curvature = 2.0 * x / (dx*dx+dy*dy);
		
		return Double.isNaN(curvature) ? 0 : curvature;
	}
	
	
	private class Path {

		private final LinkedList<Segment> segments = new LinkedList<>();
		private final List<Waypoint> waypoints;
		private final Waypoint closestPoint = new Waypoint();

		public Path(List<Waypoint> waypoints) {
			this.waypoints = waypoints;
		}

		private List<Waypoint> getWaypoints() {
			return new ArrayList<>(waypoints);
		}

		private void start(CompletionListener listener) {
			start(listener, getWaypoints());
		}

		private void start(CompletionListener listener, List<Waypoint> waypoints) {
			// create the segments from the list of waypoints
			Waypoint prev = null;
			for (Waypoint next : waypoints) {
				if (prev != null) {
					segments.add(new Segment(prev, next));
//					logger.log(Level.INFO, prev.toString());
				}
				prev = next;
			}
//			logger.log(Level.INFO, prev.toString());
			prev.addCompletionListener(listener);
		}

		/**
		 * Calculate a new look ahead waypoint which is on the path positioned the
		 * provided distance from the current location.
		 * 
		 */
		private Waypoint getLookAheadPoint(double lookAheadDistance) {

			Segment segment = null;

			ListIterator<Segment> iter = segments.listIterator();
			while (iter.hasNext()) {
				segment = iter.next();

				double remainingDistance = segment.getRemainingLength();

				// current segment long enough
				if (lookAheadDistance < remainingDistance) {
					break;
				}

				// subtract remaining distance of segment and
				// get the additional from the next segment
				if (iter.hasNext()) {
					lookAheadDistance -= remainingDistance;
				}
			}

			// return look ahead point
			return segment == null ? new Waypoint() : segment.getPoint(lookAheadDistance);
		}

		/**
		 * Update the progress of the current path segment. Advance to the next path
		 * segment if current path is complete. Return the cross track error.
		 * 
		 * @param currentPosition
		 * @return distance from path
		 */
		private double update(Pose2D currentPosition) {

			ListIterator<Segment> iter = segments.listIterator();
			while (iter.hasNext()) {

				Segment segment = iter.next();
				Waypoint closest = segment.update(currentPosition);
				this.closestPoint.x = closest.x;
				this.closestPoint.y = closest.y;

				if (segment.isComplete()) {
					// System.out.println("removing size=" + segments.size());
					iter.remove();

					// loop around to test the next segment
					continue;
				}

				return segment.getPathError();
			}

			return 0;
		}

		private Waypoint getClosestPoint() {
			return closestPoint;
		}

		/**
		 * Cycle through the path segments, adding up the segments remaining lengths.
		 */
		private double getRemainingDistance() {
			double distance = 0;
			ListIterator<Segment> iter = segments.listIterator();
			while (iter.hasNext()) {
				distance += iter.next().getRemainingLength();
			}
			return distance;
		}

	}
	
	private class Segment {

		private final Waypoint start;
		private final Waypoint end;

		private double progress = 0;
		//private boolean first = true;
		private boolean complete = false;
		private final double dx;
		private final double dy;
		private final double lengthSquared;
		private final double length;
		private final Waypoint closest = new Waypoint();
		private double pathError = 0;

		private Segment(Waypoint start, Waypoint end) {
			this.start = start;
			this.end = end;
			dx = end.x - start.x;
			dy = end.y - start.y;
			lengthSquared = dx * dx + dy * dy;
			length = Math.sqrt(lengthSquared);
		}

		private Waypoint update(Pose2D currentPosition) {

			// determine closest path location
			double dx = currentPosition.x - start.x;
			double dy = currentPosition.y - start.y;
			
			double calculatedProgress = (this.dx * dx + this.dy * dy) / lengthSquared;
			
			progress = calculatedProgress;
			
			calculatedProgress = Math.max(0.0, calculatedProgress);
			closest.x = start.x + calculatedProgress * this.dx;
			closest.y = start.y + calculatedProgress * this.dy;

			// determine distance to path
			double xerror = closest.x - currentPosition.x;
			double yerror = closest.y - currentPosition.y;
			pathError = Math.sqrt(xerror * xerror + yerror * yerror);

			// check for waypoint capture/completion
			if (calculatedProgress >= 1.0) {
				complete = true;
				end.fireCaptureEvent();
			}
			return closest;
		}

		private boolean isComplete() {
			return complete;
		}

		private double getPathError() {
			return pathError;
		}

		private double getRemainingLength() {
			return (1.0 - progress) * length;
		}

		private Waypoint getPoint(double distance) {
			double portion = progress + distance / length;

			Waypoint pt = new Waypoint();
			pt.x = start.x + portion * this.dx;
			pt.y = start.y + portion * this.dy;

			return pt;
		}
	}

}
