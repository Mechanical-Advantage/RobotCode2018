package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Runs a motion profile on the rio
 */
public class RunMotionProfileOnRioFromFile extends Command {
	
	private RunMotionProfileOnRio command;
	private boolean flipLeftRight;
	private boolean absHeading;
	private boolean backwards;
	private String filename;
	private boolean initialized = false;

    public RunMotionProfileOnRioFromFile(String filename, boolean flipLeftRight, boolean absHeading, boolean backwards) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		this.flipLeftRight = flipLeftRight;
    		this.absHeading = absHeading;
    		this.backwards = backwards;
    		this.filename = filename;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		if (!initialized) {
    			File file = new File("/home/lvuser/motionprofiles/" + filename + ".traj");
    			Trajectory trajectory = Pathfinder.readFromFile(file);
    			command = new RunMotionProfileOnRio(trajectory, flipLeftRight, absHeading, backwards);
    			initialized = true;
    		}
    		command.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !command.isRunning();
    }

    // Called once after isFinished returns true
    protected void end() {
    		command.cancel();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
}
