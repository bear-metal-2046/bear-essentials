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
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.command.Command;

public class PathActions {

	public static class PathAction {
		public final Command command;
		public final double position;
		public final boolean waitForCompletion;
		private double pathPositionStart;
		
		public PathAction(Command action) {
			this(action, 1.0, true);
		}
		
		public PathAction(Command action, double position) {
			this(action, position, true);
		}

		public PathAction(Command action, boolean waitForCompletion) {
			this(action, 1.0, waitForCompletion);
		}
		
		public PathAction(Command command, double position, boolean waitForCompletion) {
			this.command = command;
			this.position = Math.min(Math.max(position, 0.0), 1.0);
			this.waitForCompletion = waitForCompletion;
		}

		public void setPathPositionStart(double pathPositionStart) {
			this.pathPositionStart = pathPositionStart;
		}
		
		public double getPathPositionStart() {
			return pathPositionStart;
		}
	}
		
	private final List<PathAction> pathActions = new ArrayList<>();
	private final List<PathAction> pendingPathActions = new ArrayList<>();
	private final List<PathAction> startedPathActions = new ArrayList<>();

	void setupPathActions(double startLength, double length, PathAction... actions) {
		
		for(PathAction action : actions) {
			action.setPathPositionStart(startLength + length * action.position);
			pathActions.add(action);
		}
	}
	
	public List<PathAction> getPathActions() {
		return pathActions;
	}
	
	/**
	 * Reset state of the path actions
	 */
	public void resetPathActions() {
		pendingPathActions.clear();
		pendingPathActions.addAll(pathActions);
		startedPathActions.clear();
	}
	
	/**
	 * Process path actions with the provided path position. Execute commands if triggered.
	 * 
	 * @param pathPosition - current position on path
	 */
	public void processPathActions(double pathPosition) {
		Iterator<PathAction> pendingIterator = pendingPathActions.iterator();
		while(pendingIterator.hasNext()) {
			PathAction pendingPathAction = pendingIterator.next();
			if (pathPosition >= pendingPathAction.getPathPositionStart()) {
				pendingIterator.remove();
				
				// start the action
				pendingPathAction.command.start();
				
				// wait for completion if desired
				if (pendingPathAction.waitForCompletion) {
					startedPathActions.add(pendingPathAction);
				}
			}
		}
	}
	
	/**
	 * Return true if all commands are complete
	 * 
	 * @return true if all commands have completed
	 */
	public boolean arePathActionsComplete() {
		for(PathAction pathAction : startedPathActions) {
			if (pathAction.command.isRunning()) {
				return false;
			}
		}
		return true;
	}


}
