package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class PnuematicsTestExtend2 extends InstantCommand {

    public PnuematicsTestExtend2() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.pnuematicsTest2);
    }

    // Called once when the command executes
    protected void initialize() {
    		Robot.pnuematicsTest2.extend();
    }

}
