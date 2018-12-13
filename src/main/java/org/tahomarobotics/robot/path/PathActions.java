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

/**
 * PathActions holds and processes all Path Action associated with a give path.  It monitors
 * and initiates the actions on the distance trigger and monitors for completion.
 *
 */
public class PathActions {

	/**
	 * PathAction holds that state of the individual path action
	 *
	 */
	public static class PathAction {
		
		// command to execute as part of action
		public final Command command;
		
		// trigger position for path segment
		public final double position;
		
		// indicates if the action should be waited on for completion
		public final boolean waitForCompletion;
		
		// absolute trigger position
		private double pathPositionStart;
		
		/**
		 * Creates a path action for the given command to start at the end of the associated 
		 * path segment and by default it will wait for completion
		 * 
		 * @param action - command to be executed
		 */
		public PathAction(Command action) {
			this(action, 1.0, true);
		}
		
		/**
		 * Creates a path action for the given command to start at a mid-point through the associated 
		 * path segment and by default it will wait for completion
		 *  
		 * @param action - command to be executed
		 * @param position - relative mid-point position (0.0 - 1.0) 
		 */
		public PathAction(Command action, double position) {
			this(action, position, true);
		}

		/**
		 * Creates a path action for the given command to start at the end of the associated 
		 * path segment and wait for completion if desired
		 * 
		 * @param action - command to be executed
		 * @param waitForCompletion - false indicated not to wait for the action to complete
		 */
		public PathAction(Command action, boolean waitForCompletion) {
			this(action, 1.0, waitForCompletion);
		}
		
		/**
		 * Creates a path action for the given command to start at a mid-point through the associated 
		 * path segment and wait for completion if desired
		 *  
		 * @param command - command to be executed
		 * @param position - relative mid-point position (0.0 - 1.0) 
		 * @param waitForCompletion - false indicated not to wait for the action to complete
		 */
		public PathAction(Command command, double position, boolean waitForCompletion) {
			this.command = command;
			this.position = Math.min(Math.max(position, 0.0), 1.0);
			this.waitForCompletion = waitForCompletion;
		}
	}
	
	// full list of path actions
	private final List<PathAction> pathActions = new ArrayList<>();
	
	// list of path actions that are waiting to be executed
	private final List<PathAction> pendingPathActions = new ArrayList<>();
	
	// list of path actions that have been started that will be waited on to complete
	private final List<PathAction> startedPathActions = new ArrayList<>();

	/**
	 * Determines the actual action path length and adds the updated action to the list
	 * 
	 * @param startLength - path segment start
	 * @param length - path segment length
	 * @param actions - action to be added
	 */
	protected void setupPathActions(double startLength, double length, PathAction... actions) {
		for(PathAction action : actions) {
			action.pathPositionStart = startLength + length * action.position;
			pathActions.add(action);
		}
	}
	
	/**
	 * Returns the list of PathActions
	 * 
	 * @return List of PathActions
	 */
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
			if (pathPosition >= pendingPathAction.pathPositionStart) {
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
