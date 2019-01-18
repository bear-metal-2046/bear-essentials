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

import org.tahomarobotics.robot.state.Pose2D;
import org.tahomarobotics.robot.util.MathUtil;

public class PathSection {
	public final double length;
	public final double angle;
	public final double radius;
	public final double maxVelocity;
	public double maxRotationalVelocity;
	public final Pose2D startPose;
	public final Pose2D endPose;
	
	
	public PathSection(double length, double maxVelocity, Pose2D startPose) {
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
	
	public PathSection(double angle, double radius, double maxVelocity, Pose2D startPose) {
		double angleRadians = MathUtil.normalizeAngle(Math.toRadians(angle));
		this.length = Math.abs(angleRadians) * radius;
		this.angle = angle;
		this.radius = radius;
		this.maxVelocity = maxVelocity;
		this.startPose = startPose;
		this.endPose =  new Pose2D(startPose);		

		double chord = 2.0 * radius * Math.sin(Math.abs(angleRadians)/2.0) * (Math.abs(angle) > 180 ? -1 : 1);
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
