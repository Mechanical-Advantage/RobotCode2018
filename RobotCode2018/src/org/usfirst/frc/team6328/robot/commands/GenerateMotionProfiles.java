package org.usfirst.frc.team6328.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team6328.robot.RobotMap;

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
	public static final int waypointVersion = 27;
	
	private final Trajectory.Config stdConfig2017 = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC,
			Trajectory.Config.SAMPLES_HIGH, 0.02, /*100*/25, /*55*/40, 200); // jerk actually matters
	private final Trajectory.Config stdConfigPractice = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, 
			Trajectory.Config.SAMPLES_HIGH, 0.02, 100, 30, 200);
	@SuppressWarnings("unused")
	private Trajectory.Config config; // this can be defined for specific profiles
	private final String MPDir = "/home/lvuser/motionprofiles/"; // make sure to have slash at end
	private final double robotLength = 31.5;
	private final double robotWidth = 29.25;
	
	// Defined points on field
	private final Waypoint sideStart = new Waypoint(robotLength/2, 132-robotWidth/2, 0);
	private final Waypoint switchSide = new Waypoint(168, 76.75+(robotLength/2), Pathfinder.d2r(-90));
	private final Waypoint switchFront = new Waypoint(168-28-(robotLength/2), 54, 0);
	@SuppressWarnings("unused")
	private final Waypoint switchBack = new Waypoint(168+28+(robotLength/2), 54, Pathfinder.d2r(180));
	private final Waypoint scaleFront = new Waypoint(300-(robotLength/2), 90-12, 0);
	private final Waypoint scaleFrontOpposite = new Waypoint(scaleFront.x, scaleFront.y*-1, 0);
	private final Waypoint sideSwitchPrepareCrossingPoint = new Waypoint(168+36, 76.75+42.625, Pathfinder.d2r(-135));
	
	Waypoint[] points;

    public GenerateMotionProfiles() {
        super("GenerateMotionProfiles");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
	    	new File(MPDir).mkdir(); // will do nothing if directory exists, but make sure it is there
	    	
	    	// these waypoints should be defined assuming the right side of the field, auto flipped on left
	    	// 0,0 is center of field along starting wall
	    	// Points are defined from robot center
	    	
	    	points = new Waypoint[] {
	    			new Waypoint(0, 0, 0),
	    			new Waypoint(120, 60, Pathfinder.d2r(90))
	    	};
	    	generateProfile("test10forward5right");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
//	    			new Waypoint(168-42.875, 150-robotWidth/2, 0),
	    			new Waypoint((168-42.875)+21.4375, 76.75+(robotLength/2)+36.37, Pathfinder.d2r(-30)),
//	    			new Waypoint((168-42.875)+36.37, 76.75+(robotLength/2)+21.4375, Pathfinder.d2r(-60)),
	    			switchSide
	    	};
	    	generateProfile("sideToSwitch");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			switchFront
	    	};
	    	generateProfile("sideToSwitchFront");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			new Waypoint(168, 76.75+18+(robotWidth/2), 0), // Left edge of robot 18 in. from switch
	    			scaleFront
	    	};
	    	generateProfile("sideToScale");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			new Waypoint(168, 76.75+36+(robotWidth/2), 0), // Left edge of robot 36 in. from switch
	    			new Waypoint(168+28+25.5, 0, Pathfinder.d2r(-90)),
	    			scaleFrontOpposite
	    	};
	    	generateProfile("sideToOppositeScale");
	    	
	    	points = new Waypoint[] {
	    			switchSide,
	    			sideSwitchPrepareCrossingPoint
	    	};
	    	generateProfile("sideSwitchPrepareCrossing");
	    	
	    	points = new Waypoint[] {
	    			sideSwitchPrepareCrossingPoint,
	    			new Waypoint(228.735, -24, Pathfinder.d2r(-90)) // -24 is a guess
	    	};
	    	generateProfile("sideSwitchCross");
	    	
	    	points = new Waypoint[] {
	    			scaleFront,
	    			new Waypoint(228.735, 150-(robotLength/2), Pathfinder.d2r(-90)) // 150 is a guess
	    	};
	    	generateProfile("scalePrepareCrossing");
	    	
	    	points = new Waypoint[] {
	    			switchSide,
//	    			new Waypoint(168-42.875, 150-robotWidth/2, 0),
	    			new Waypoint((168-42.875)+21.4375, 76.75+(robotLength/2)+36.37, Pathfinder.d2r(-30)),
//	    			new Waypoint((168-42.875)+36.37, 76.75+(robotLength/2)+21.4375, Pathfinder.d2r(-60)),
	    			sideStart
	    	};
	    	generateProfile("backwardsTest");
    	
//    	Example custom config
//    	config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
//    			Trajectory.Config.SAMPLES_HIGH, 0.05, /*124.4*/120, /*62.2,41.4*/55, /*1866*/2700);
//    	points = new Waypoint[] {
//    			new Waypoint(0, 0, 0),
//    			new Waypoint(24, -48, Pathfinder.d2r(-60)),
//    			new Waypoint(48, -96, Pathfinder.d2r(-60)),
//    	};
//    	generateProfile(config, "sideCrossWhiteLine");
    	
    	
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

    private void generateProfile(String fileName) {
    		switch (RobotMap.robot) {
			case EVERYBOT_2018:
				break;
			case ORIGINAL_ROBOT_2018:
				break;
			case PRACTICE:
				generateProfile(stdConfigPractice, fileName);
				break;
			case ROBOT_2017:
				generateProfile(stdConfig2017, fileName);
				break;
    		}
    }
    
    private void generateProfile(Trajectory.Config config, String fileName) {
	    	System.out.println("Generating Profile " + fileName + "...");
	    	Trajectory trajectory = Pathfinder.generate(points, config);
	    	File file = new File(MPDir + fileName + ".traj");
	    	Pathfinder.writeToFile(file, trajectory);
	    	File CSVfile = new File(MPDir + fileName + ".csv");
	    	Pathfinder.writeToCSV(CSVfile, trajectory);
    }
}
