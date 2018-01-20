package org.usfirst.frc.team6328.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.command.InstantCommand;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * Generates a set of motion profiles and saves them to a file for the auto sequences
 */
public class GenerateMotionProfiles extends InstantCommand {
	
	// IMPORTANT!
	// increment this by 1 every time the waypoints are changed
	// the robot will re-generate profiles if this is greater than saved
	public static final int waypointVersion = 1;
	
	private final Trajectory.Config stdConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC,
			Trajectory.Config.SAMPLES_HIGH, 0.02, 100, 55, 2700);
	@SuppressWarnings("unused")
	private Trajectory.Config config; // this can be defined for specific profiles
	private final String MPDir = "/home/lvuser/motionprofiles/"; // make sure to have slash at end

    public GenerateMotionProfiles() {
        super("GenerateMotionProfiles");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	new File(MPDir).mkdir(); // will do nothing if directory exists, but make sure it is there
    	
    	Waypoint[] points;
    	
    	// these waypoints should be defined assuming the left side of the field, auto flipped on right
    	
    	points = new Waypoint[] {
    			new Waypoint(0, 0, 0),
    			new Waypoint(120, 60, Pathfinder.d2r(90))
    	};
    	generateProfile(points, "test10forward5right");
    	
//    	Example custom config
//    	config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
//    			Trajectory.Config.SAMPLES_HIGH, 0.05, /*124.4*/120, /*62.2,41.4*/55, /*1866*/2700);
//    	points = new Waypoint[] {
//    			new Waypoint(0, 0, 0),
//    			new Waypoint(24, -48, Pathfinder.d2r(-60)),
//    			new Waypoint(48, -96, Pathfinder.d2r(-60)),
//    	};
//    	generateProfile(points, config, "sideCrossWhiteLine");
    	
    	
    	// write the current waypoint version to a file
    	try {
			BufferedWriter fileWriter = new BufferedWriter(new FileWriter("/home/lvuser/lastWaypointVersion"));
			fileWriter.write(String.valueOf(waypointVersion));
			fileWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
			System.out.println("File writing failed, profile generation will happen again next time, error above");
		}
    	
    	
    	System.out.println("All profiles generated");
    }

    private void generateProfile(Waypoint[] points, String fileName) {
    		generateProfile(points, stdConfig, fileName);
    }
    
    private void generateProfile(Waypoint[] points, Trajectory.Config config, String fileName) {
	    	System.out.println("Generating Profile " + fileName + "...");
	    	Trajectory trajectory = Pathfinder.generate(points, config);
	    	File file = new File(MPDir + fileName + ".traj");
	    	Pathfinder.writeToFile(file, trajectory);
	    	File CSVfile = new File(MPDir + fileName + ".csv");
	    	Pathfinder.writeToCSV(CSVfile, trajectory);
    }
}
