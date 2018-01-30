package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class PnuematicsTestRelease extends InstantCommand {

    public PnuematicsTestRelease() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.pnuematicsTest);
    }

    // Called once when the command executes
    protected void initialize() {
    		Robot.pnuematicsTest.neutral();
    }

}
