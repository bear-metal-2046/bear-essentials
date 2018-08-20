package org.tahomarobotics.robot.path;

import org.tahomarobotics.robot.state.Pose2D;

public interface PathController {

	double update(Pose2D pose);

}
