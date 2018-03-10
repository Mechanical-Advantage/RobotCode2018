package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Set the target elevator position. 
 */
public class SetElevatorPosition extends Command {
	
	private static final double bangBangSpeed = 1;
	
	private ElevatorPosition targetPosition;
	private boolean enableFinish;
	private static final boolean motionMagic = false;
	
	public SetElevatorPosition(ElevatorPosition position) {
		this(position, true);
	}

	public SetElevatorPosition(ElevatorPosition position, boolean enableFinish) {
		super("SetElevatorPosition");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
		targetPosition = position;
		this.enableFinish = enableFinish;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.switchGear(ElevatorGear.HIGH);
		double targetPositionInches = Robot.elevator.getPositionTarget(targetPosition);
		if (motionMagic) {
			Robot.elevator.setPosition(targetPositionInches);
		} else {
			if (targetPositionInches < Robot.elevator.getPosition()) {
				Robot.elevator.driveOpenLoop(bangBangSpeed*-1);
			} else if (targetPositionInches > Robot.elevator.getPosition()) {
				Robot.elevator.driveOpenLoop(bangBangSpeed);
			}
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		boolean value = Robot.elevator.onTarget(Robot.elevator.getPositionTarget(targetPosition));
		if (enableFinish) {
			return value;
		} else if (value) {
			Robot.elevator.holdPosition();
		}
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
