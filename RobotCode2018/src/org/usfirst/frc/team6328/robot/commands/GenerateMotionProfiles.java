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
	public static final int waypointVersion = 90;
	
	private final Trajectory.Config stdConfig2017 = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC,
			Trajectory.Config.SAMPLES_HIGH, 0.02, 100, 55, 200); // jerk actually matters
	private final Trajectory.Config stdConfigPractice = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, 
			Trajectory.Config.SAMPLES_HIGH, 0.02, 100, 30, 200);
	private final Trajectory.Config stdConfigOrig2018 = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, 
			Trajectory.Config.SAMPLES_HIGH, 0.02, 150, 30, 300);
	
	// Constants for field positions
	private static final double switchCenterDistance = 168; // Distance from wall to center of switch
	private static final double switchWidth = 153.5;
	private static final double halfSwitchWidth = switchWidth/2;
	private static final double scaleWidth = 180;
	private static final double halfScaleWidth = scaleWidth/2;
	private static final double scaleCenterDistance = 299.65+(48/2);
	private static final double plateCenterDistance = 54; // Distance from center of field to center of plates on switch
	private static final double exchangeRampOffset = -12; // Distance from center of field of edge of exchange ramp
	
	@SuppressWarnings("unused")
	private Trajectory.Config config; // this can be defined for specific profiles
	private final String MPDir = "/home/lvuser/motionprofiles/"; // make sure to have slash at end
	
	// Defined points on field
	private Waypoint sideStart;
	private Waypoint centerStart;
	private Waypoint switchSide;
	private Waypoint switchSideIn;
	private Waypoint switchFront;
	private Waypoint switchFrontIn;
	private Waypoint switchFrontOpposite;
	private Waypoint switchFrontOppositeIn;
	@SuppressWarnings("unused")
	private Waypoint switchBack;
	private Waypoint scaleFront;
	@SuppressWarnings("unused")
	private Waypoint scaleFrontOpposite;
	private Waypoint scaleSide;
	@SuppressWarnings("unused")
	private Waypoint sideSwitchPrepareCrossingPoint;
	private Waypoint alongSwitch;
	private Waypoint alongSwitchFar;
	
	Waypoint[] points;

    public GenerateMotionProfiles() {
        super("GenerateMotionProfiles");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        switch (RobotMap.robot) {
		case EVERYBOT_2018:
			break;
		case ORIGINAL_ROBOT_2018:
			break;
		case PRACTICE:
			break;
		case ROBOT_2017:
			break;
        }
        defineWaypoints();
    }
    
    private void defineWaypoints() {
	    	 sideStart = new Waypoint(RobotMap.robotLength/2, 132-RobotMap.robotWidth/2, 0);
	    	 centerStart = new Waypoint(RobotMap.robotLength/2, exchangeRampOffset+(RobotMap.robotWidth/2), 0);
	    	 switchSide = new Waypoint(switchCenterDistance, halfSwitchWidth+(RobotMap.robotLength/2), Pathfinder.d2r(-90));
	    	 switchSideIn = new Waypoint(switchSide.x, switchSide.y-12, switchSide.angle); // 1 foot into switch
	    	 switchFront = new Waypoint(switchCenterDistance-28-(RobotMap.robotLength/2), plateCenterDistance, 0);
	    	 switchFrontIn = new Waypoint(switchFront.x+12, switchFront.y, switchFront.angle); // 1 foot into switch
	    	 switchFrontOpposite = new Waypoint(switchCenterDistance-28-(RobotMap.robotLength/2), plateCenterDistance*-1, 0);
	    	 switchFrontOppositeIn = new Waypoint(switchFrontOpposite.x+12, switchFrontOpposite.y, switchFrontOpposite.angle); // 1 foot into switch
	    	 switchBack = new Waypoint(switchCenterDistance+28+(RobotMap.robotLength/2), plateCenterDistance, Pathfinder.d2r(180));
	    	 scaleFront = new Waypoint(300-(RobotMap.robotLength/2), 90-12, Pathfinder.d2r(-25));
	    	 scaleFrontOpposite = new Waypoint(scaleFront.x, scaleFront.y*-1, Pathfinder.d2r(3));
	    	 scaleSide = new Waypoint(scaleCenterDistance, halfScaleWidth+(RobotMap.robotLength/2), Pathfinder.d2r(-90));
	    	 sideSwitchPrepareCrossingPoint = new Waypoint(switchCenterDistance+120, halfSwitchWidth+32, Pathfinder.d2r(/*-135*/180));
	    	 alongSwitch = new Waypoint(switchCenterDistance, halfSwitchWidth+18+(RobotMap.robotWidth/2), 0); // Left edge of robot 18 in. from switch
	    	 alongSwitchFar = new Waypoint(switchCenterDistance, halfSwitchWidth+36+(RobotMap.robotWidth/2), 0); // Left edge of robot 36 in. from switch
    }

    // Called once when the command executes
    protected void initialize() {
	    	new File(MPDir).mkdir(); // will do nothing if directory exists, but make sure it is there
	    	
	    	// these waypoints should be defined assuming the right side of the field, auto flipped on left
	    	// 0,0 is center of field along starting wall
	    	// Points are defined from robot center
	    	
//	    	points = new Waypoint[] {
//	    			new Waypoint(0, 0, 0),
//	    			new Waypoint(120, 60, Pathfinder.d2r(90))
//	    	};
//	    	generateProfile("test10forward5right");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
//	    			new Waypoint(switchCenterDistance-42.875, 150-RobotMap.robotWidth/2, 0),
	    			new Waypoint((switchCenterDistance-42.875)+21.4375, halfSwitchWidth+(RobotMap.robotLength/2)+36.37, Pathfinder.d2r(-30)),
//	    			new Waypoint((switchCenterDistance-42.875)+36.37, halfSwitchWidth+(RobotMap.robotLength/2)+21.4375, Pathfinder.d2r(-60)),
	    			switchSideIn
	    	};
	    	generateProfile("sideToSwitch");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			alongSwitchFar,
	    			new Waypoint(/*switchCenterDistance+28+25.5*/230, 0, Pathfinder.d2r(-90)),
	    			new Waypoint(/*switchCenterDistance+28+25.5*/230, -66, Pathfinder.d2r(-90)),
	    			new Waypoint(switchCenterDistance+36, (halfSwitchWidth+(RobotMap.robotWidth/2)+28)*-1, Pathfinder.d2r(180)),
	    			new Waypoint(switchCenterDistance, (halfSwitchWidth+(RobotMap.robotWidth/2)+28)*-1, Pathfinder.d2r(180)) // Edge of robot 28 in. from switch
	    	};
	    	generateProfile("sideToOppositeSwitch");
	    	
//	    	points = new Waypoint[] {
//	    			sideStart,
//	    			switchFront
//	    	};
//	    	generateProfile("sideToSwitchFront");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			alongSwitch,
	    			scaleFront
	    	};
	    	generateProfile("sideToScale");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			alongSwitchFar,
	    			new Waypoint(/*switchCenterDistance+28+25.5*/230, 30, Pathfinder.d2r(-90)),
	    			new Waypoint(/*switchCenterDistance+28+25.5*/230, 0, Pathfinder.d2r(-90)),
	    			new Waypoint(/*switchCenterDistance+28+25.5*/230, -60, Pathfinder.d2r(-90)),
	    			scaleFrontOpposite
	    	};
	    	generateProfile("sideToOppositeScale");
	    	
	    	points = new Waypoint[] {
	    			sideStart,
	    			new Waypoint(192, sideStart.y+12, Pathfinder.d2r(5)),
	    			new Waypoint(scaleCenterDistance-21.4375-(RobotMap.robotLength/2)+12, sideStart.y+16, Pathfinder.d2r(-20)),
	    			scaleSide
	    	};
	    	generateProfile("sideToScaleEnd");
	    	
	    	points = new Waypoint[] {
	    			switchSide,
	    			new Waypoint((switchCenterDistance+42.875)-21.4375, halfSwitchWidth+(RobotMap.robotLength/2)+15, Pathfinder.d2r(-150)),
	    			sideSwitchPrepareCrossingPoint
	    	};
	    	generateProfile("sideSwitchPrepareCrossing");
	    	
	    	points = new Waypoint[] {
	    			sideSwitchPrepareCrossingPoint,
	    			new Waypoint(228.735, scaleFrontOpposite.y-24, Pathfinder.d2r(-90))
	    	};
	    	generateProfile("sideSwitchCross");
	    	
	    	points = new Waypoint[] {
	    			scaleFront,
	    			new Waypoint(228.735, 135-(RobotMap.robotLength/2), Pathfinder.d2r(-90)) // 150 is a guess
	    	};
	    	generateProfile("scalePrepareCrossing");
//	    	
//	    	points = new Waypoint[] {
//	    			switchSide,
////	    			new Waypoint(168-42.875, 150-RobotMap.robotWidth/2, 0),
//	    			new Waypoint((168-42.875)+21.4375, 76.75+(RobotMap.robotLength/2)+36.37, Pathfinder.d2r(-30)),
////	    			new Waypoint((168-42.875)+36.37, 76.75+(RobotMap.robotLength/2)+21.4375, Pathfinder.d2r(-60)),
//	    			sideStart
//	    	};
//	    	generateProfile("backwardsTest");
//	    	
	    	points = new Waypoint[] {
	    			centerStart,
	    			switchFrontIn
	    	};
	    	generateProfile("centerToRightSwitch");
	    	
	    	points = new Waypoint[] {
	    			centerStart,
	    			switchFrontOppositeIn
	    	};
	    	generateProfile("centerToLeftSwitch");
	    	
//	    	points = new Waypoint[] {
//	    			new Waypoint(0, 0, 0),
//	    			new Waypoint(8*12, 0, 0)
//	    	};
//	    	generateProfile("8straight");
    	
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
				generateProfile(stdConfigOrig2018, fileName);
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
