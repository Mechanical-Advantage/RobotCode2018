package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * Runs a motion profile from a file
 */
public class RunMotionProfileOnRioFromFile extends CommandGroup {
	
	String filename;
	boolean flipLeftRight;
	boolean absHeading;
	
	boolean initialized = false;

    public RunMotionProfileOnRioFromFile(String filename, boolean flipLeftRight, boolean absHeading) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    		this.filename = filename;
    		this.flipLeftRight = flipLeftRight;
    		this.absHeading = absHeading;
    }
    
    @Override
    protected void initialize() {
    		if (!initialized) {
    			File file = new File("motionprofiles/" + filename);
    			Trajectory trajectory = Pathfinder.readFromFile(file);
    			addSequential(new RunMotionProfileOnRio(trajectory, flipLeftRight, absHeading));
    		}
    }
}
