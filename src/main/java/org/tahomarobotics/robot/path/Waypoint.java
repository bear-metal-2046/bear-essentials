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

import org.tahomarobotics.robot.state.Pose2D;

public class Waypoint {
	
	private final List<CompletionListener> listeners = new ArrayList<>();

	public double x;
	public double y;
	public double speed;

	public Waypoint() {	
	}
	
	public Waypoint(Waypoint src) {
		x = src.x;
		y = src.y;
		speed = src.speed;
	}
	
	public Waypoint(Pose2D src, double speed) {
		x = src.x;
		y = src.y;
		this.speed = speed;
	}
	
	public Waypoint(double x, double y, double speed) {
		this(x, y, speed, null);
	}
	
	public Waypoint(double x, double y, double speed, CompletionListener listener) {
		this.x = x;
		this.y = y;
		this.speed = speed;
		if (listener != null) {
			this.addCompletionListener(listener);
		}
	}
	
	public void addCompletionListener(CompletionListener listener) {
		listeners.add(listener);
	}

	protected void fireCaptureEvent() {
		for (CompletionListener listener : listeners) {
			listener.onCompletion();
		}
	}

	protected Waypoint mult(double a) {
		return new Waypoint(x*a, y*a, speed);
	}

	protected Waypoint add(Waypoint other) {
		return new Waypoint(x+other.x,y+other.y,speed);
	}

	protected Waypoint div(double a) {
		return new Waypoint(x/a, y/a, speed);
	}
	
	public double distance(Waypoint pt) {
		return distance(pt.x, pt.y);
	}
	
	public double distance(double x, double y) {
		double dx = x - this.x;
		double dy = y - this.y;
		return Math.sqrt(dx*dx+dy*dy);
	}
	
	public double angle(Waypoint pt) {
		return angle(pt.x, pt.y);
	}
	
	public double angle(double x, double y) {
		double dx = this.x - x;
		double dy = this.y - y;
		return Math.atan2(dy, dx);
	}

	@Override
	public String toString() {
		return String.format("%6.3f %6.3f %6.3f", x, y, speed);
	}
}
