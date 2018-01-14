package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class PnuematicsTestExtend extends InstantCommand {

    public PnuematicsTestExtend() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.pnuematicsTest);
    }

    // Called once when the command executes
    protected void initialize() {
    		Robot.pnuematicsTest.extend();
    }

}
