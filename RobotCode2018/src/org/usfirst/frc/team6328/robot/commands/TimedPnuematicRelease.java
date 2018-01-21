package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TimedPnuematicRelease extends Command {
	
	Timer timer = new Timer();

    public TimedPnuematicRelease() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.pnuematicsTest);
    		requires(Robot.pnuematicsTest2);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.pnuematicsTest.extend();
    		timer.reset();
    		timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		if (timer.get() >= SmartDashboard.getNumber("Pnuematic Timer", 1.0)) {
			Robot.pnuematicsTest2.retract();
			return true;
		}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		System.out.println(timer.get());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
