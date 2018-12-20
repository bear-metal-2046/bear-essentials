package org.tahomarobotics.robot.path.follower;

import org.tahomarobotics.robot.motion.MotionState;
import org.tahomarobotics.robot.path.Path;
import org.tahomarobotics.robot.path.Waypoint;
import org.tahomarobotics.robot.state.Pose2D;
import org.tahomarobotics.robot.state.RobotSpeed;

import java.util.List;

public class RamseteController {

    private double zeta, b, k1, errX, errY, errTheta, pathHeading;
    private Path path;
    private boolean complete = false;

    private static double D_ZETA = 1.0;
    private static double D_B = 0.001;
    private static double EPISILON = 0.000001;

    public RamseteController(List<Waypoint> waypoints) {
        this(D_ZETA, D_B, waypoints);
    }

    public RamseteController(double zeta, double b, List<Waypoint> waypoints) {
        this.zeta = zeta;
        this.b = b;
        path = new Path(waypoints);

        path.start(() -> {
            System.out.println("Finished Path");
            complete = true;
        });
    }

    /**
     * Fills the robot speed with the wanted velocities based off of Ramsete: http://www.diag.uniroma1.it/%7Elabrob/pub/papers/RAMSETE_Chap_LNCIS270.pdf
     *
     * @param toFill         RobotSpeed
     * @param wantedForward  MotionState
     * @param wantedRotation MotionState
     * @param pose           Pose2D
     */
    public void update(RobotSpeed toFill, MotionState wantedForward, MotionState wantedRotation, Pose2D pose) {
        //Get point we should be at
        Waypoint point = path.getPointFromDistance(wantedForward.position);
        Path.Segment segment = path.getSegmentFromDistance(wantedForward.position);

        pose.heading = boundHeading(pose.heading);

        pathHeading = boundHeading(Math.atan2(segment.dy, segment.dx));

        //System.out.format("DX: %6.3f DY: %6.3f HEADING: %6.3f", segment.dx, segment.dy, pathHeading);

        //Calculate k1 as 2 * zeta * a
        k1 = 2 * zeta * Math.sqrt(Math.pow(wantedRotation.velocity, 2) + b * Math.pow(wantedForward.velocity, 2));

        //Error in relation to the Robots position and heading, x'y' Cartesian coordinates
        errX = Math.cos(pose.heading) * (point.x - pose.x) + Math.sin(pose.heading) * (point.y - pose.y);
        errY = Math.cos(pose.heading) * (point.y - pose.y) - Math.sin(pose.heading) * (point.x - pose.x);
        errTheta = pathHeading - pose.heading;

        //Calculate commanded forward Velocity
        toFill.forward = wantedForward.velocity * Math.cos(errTheta) + k1 * errX;

        //Calculate command rotational Velocity - k1 is k3 in the paper
        toFill.rotational = wantedRotation.velocity + b * wantedForward.velocity * sinThetaOverTheta() * errY + k1 * errTheta;

        //System.out.format("Wanted forward velocity: %6.3f Wanted Rotation: %6.3f k1: %6.3f errX: %6.3f errY: %6.3f errTheta: %6.3f Motion Forward: %6.3f Motion Rotation: %6.3f\n", toFill.forwardVelocity, toFill.rotationalVelocity, k1, errX, errY, errTheta, wantedForward.velocity, wantedRotation.velocity);
    }

    private double boundHeading(double heading) {
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;
        return heading;
    }

    private double sinThetaOverTheta() {
        if (Math.abs(errTheta) < EPISILON) {
            return 1.0;
        } else {
            return Math.sin(errTheta) / errTheta;
        }
    }
}