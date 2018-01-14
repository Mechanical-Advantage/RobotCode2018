package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Sets whether the joysticks are reversed in OI
 */
public class ReverseJoysticks extends InstantCommand {
	
	private boolean reverseJoysticks;

    public ReverseJoysticks(boolean reverse) {
        super("ReverseJoysticks");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        reverseJoysticks = reverse;
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.oi.reverseJoysticks(reverseJoysticks);
    	SmartDashboard.putString("Joysticks Reversed", reverseJoysticks ? "Reversed" : "Normal");
    }

}
