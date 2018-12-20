package org.tahomarobotics.robot.path;

import org.tahomarobotics.robot.state.Pose2D;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

public class Path {

    private final LinkedList<Segment> segments = new LinkedList<>();
    private final List<Waypoint> waypoints;
    private final Waypoint closestPoint = new Waypoint();

    public Path(List<Waypoint> waypoints) {
        this.waypoints = waypoints;
    }

    private List<Waypoint> getWaypoints() {
        return new ArrayList<>(waypoints);
    }

    public void start(CompletionListener listener) {
        start(listener, getWaypoints());
    }

    public void start(CompletionListener listener, List<Waypoint> waypoints) {

        segments.clear();

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
    public Waypoint getLookAheadPoint(double lookAheadDistance) {

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
    public double update(Pose2D currentPosition) {

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

    public Waypoint getClosestPoint() {
        return closestPoint;
    }

    /**
     * Cycle through the path segments, adding up the segments remaining lengths.
     */
    public double getRemainingDistance() {
        double distance = 0;
        ListIterator<Segment> iter = segments.listIterator();
        while (iter.hasNext()) {
            distance += iter.next().getRemainingLength();
        }
        return distance;
    }

    /**
     * Calculate the distance traveled in the path given a distance
     * @param distance
     * @return Waypoint
     */
    public Waypoint getPointFromDistance(double distance){
        Segment segment = null;
        ListIterator<Segment> iter = segments.listIterator();
        while (iter.hasNext()) {
            segment = iter.next();
            double remainingDistance = segment.getRemainingLength();

            if(distance < remainingDistance){
                break;
            }

            if(iter.hasNext()){
                distance -= remainingDistance;
            }
        }

        return segment == null ? new Waypoint() : segment.getPoint(distance);
    }

    public Segment getSegmentFromDistance(double distance){
        Segment segment = null;
        ListIterator<Segment> iter = segments.listIterator();
        while (iter.hasNext()) {
            segment = iter.next();
            double remainingDistance = segment.getRemainingLength();

            if(distance < remainingDistance){
                break;
            }

            if(iter.hasNext()){
                distance -= remainingDistance;
            }
        }

        return segment == null ? new Segment(new Waypoint(0, 0, 0), new Waypoint(0, 0, 0)) : segment;
    }

    public class Segment {

        private final Waypoint start;
        private final Waypoint end;

        private double progress = 0;
        //private boolean first = true;
        private boolean complete = false;
        public final double dx;
        public final double dy;
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

