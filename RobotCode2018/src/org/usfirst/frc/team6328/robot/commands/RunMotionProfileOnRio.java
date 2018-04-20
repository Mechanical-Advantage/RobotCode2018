package org.usfirst.frc.team6328.robot.commands;

import java.io.File;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.TunableNumber;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Runs a motion profile on the roboRIO. dt of profile must be 0.02. 
 * If using absolute yaw, starting yaw must be close to first point heading. 
 * If running backwards, profile should be generated as if going forwards.
 */
public class RunMotionProfileOnRio extends Command {
	
	private final int sensorFrameRate = 2; // ms
	private final int controlFrameRate = 10;
	private final boolean enableGyroCorrection = true; // false for tuning
	
	private TunableNumber kP = new TunableNumber("Profile P"); // P and D apply when profile is next started
	private TunableNumber kD = new TunableNumber("Profile D");
	private TunableNumber AngleErrorThreshold = new TunableNumber("Profile Angle Error Threshold");
	private TunableNumber wheelbase = new TunableNumber("Wheelbase");
	private TunableNumber kPAngle = new TunableNumber("Profile Angle P"); // standard is 0.8 * (1.0/80.0), add max velocity like: 0.8*60 * (1.0/80.0) (if maxVel is 60)
	// kPAngle should be positive
	private TunableNumber kPAngleAdjust = new TunableNumber("Profile Angle Adjust P");
	private TunableNumber kV = new TunableNumber("Profile V");
	private TunableNumber kA = new TunableNumber("Profile A");
	
	private float tiltThreshold;
	private DriveGear gear;
	
	private CustomDistanceFollower leftFollower, rightFollower;
	private double initialYaw;
	private double initialProfileYaw;
	private double initialPositionLeft, initialPositionRight;
	private double gyroHeading;
	private double desiredHeading;
	private double angleDifference;
	private double positionErrorLeftPositive, positionErrorLeftNegative, positionErrorRightPositive,
		positionErrorRightNegative, yawErrorPositive, yawErrorNegative;
	private int trajectoryLength;
	private TankModifier modifier;
	private boolean flipLeftRight;
	private boolean absHeading;
	private boolean backwards;
	private ConvergenceMode endConverge;
	private boolean trajectoryLoaded = false;
	private String filename;
	
	/*
     * Tuning Notes:
     * Tune wheelbase with gyro correction disabled until heading is about accurate
     * D should be significantly lower than P
     * Too much D will overcorrect, robot will go too fast, not much is needed, in my initial test, any causes problems
     */
	
	public RunMotionProfileOnRio(Trajectory trajectory, boolean flipLeftRight, boolean absHeading, boolean backwards, boolean endConverge) {
		this(trajectory, flipLeftRight, absHeading, backwards, endConverge ? ConvergenceMode.ALWAYS : ConvergenceMode.NEVER);
}

    public RunMotionProfileOnRio(Trajectory trajectory, boolean flipLeftRight, boolean absHeading, boolean backwards, ConvergenceMode endConverge) {
    		super("RunMotionProfileOnRio");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		this.flipLeftRight = flipLeftRight;
    		this.absHeading = absHeading;
    		this.backwards = backwards;
    		this.endConverge = endConverge;
    		initCommand();
    		initTrajectory(trajectory);
    }
    
    public RunMotionProfileOnRio(String filename, boolean flipLeftRight, boolean absHeading, boolean backwards, boolean endConverge) {
    		this(filename, flipLeftRight, absHeading, backwards, endConverge ? ConvergenceMode.ALWAYS : ConvergenceMode.NEVER);
    }
    
    public RunMotionProfileOnRio(String filename, boolean flipLeftRight, boolean absHeading, boolean backwards, ConvergenceMode endConverge) {
    		this.flipLeftRight = flipLeftRight;
		this.absHeading = absHeading;
		this.backwards = backwards;
		this.filename = filename;
		this.endConverge = endConverge;
		initCommand();
    }
    
    private void initCommand() {
    		requires(Robot.driveSubsystem);
    		switch (RobotMap.robot) {
		case PRACTICE:
			kP.setDefault(10);
			kD.setDefault(0);
			AngleErrorThreshold.setDefault(1.5);
			wheelbase.setDefault(30);
			kPAngle.setDefault(2);
			kPAngleAdjust.setDefault(1);
			break;
		case ROBOT_2017:
			// tuned using max velocity: 100, accel: 55, jerk: 200
			kP.setDefault(8); // No oscillation at 10, 15 has some but D might help, 11 on 1/26
			kD.setDefault(0);
			AngleErrorThreshold.setDefault(1.5);
			wheelbase.setDefault(22); // 18 measured, 19.5 at low speed
			kPAngle.setDefault(3);
			kPAngleAdjust.setDefault(0.1);
			kV.setDefault(0.982);
			kA.setDefault(0.09);
			break;
		case ORIGINAL_ROBOT_2018:
			kP.setDefault(3);
			kD.setDefault(0);
			AngleErrorThreshold.setDefault(1.5);
			wheelbase.setDefault(25.5);
			kPAngle.setDefault(4);
			kPAngleAdjust.setDefault(0.2);
			gear = DriveGear.HIGH;
			kV.setDefault(1.11);
			kA.setDefault(0.12);
			tiltThreshold = 5;
			break;
		case EVERYBOT_2018:
			break;
		default:
			break;
    		}
    }
    
    private void initTrajectory(Trajectory trajectory) {
    		trajectoryLength = trajectory.length();
		modifier = new TankModifier(trajectory);
		leftFollower = new CustomDistanceFollower();
		rightFollower = new CustomDistanceFollower();
		initFollowers();
		trajectoryLoaded = true;
    }
    
    // In separate function because this gets run during init if in tuning mode to load new wheelbase
    private void initFollowers() {
		modifier.modify(wheelbase.get());
		Trajectory leftTrajectory;
		Trajectory rightTrajectory;
		if (backwards) {
			// If running backwards, sides should be switched
			leftTrajectory = modifier.getRightTrajectory();
			rightTrajectory = modifier.getLeftTrajectory();
			int i;
			for (i=0; i<leftTrajectory.length(); i++) {
				leftTrajectory.segments[i].heading+=Math.PI;
				leftTrajectory.segments[i].position*=-1;
				leftTrajectory.segments[i].velocity*=-1;
				leftTrajectory.segments[i].acceleration*=-1;
				leftTrajectory.segments[i].jerk*=-1;
			}
			for (i=0; i<rightTrajectory.length(); i++) {
				rightTrajectory.segments[i].heading+=Math.PI;
				rightTrajectory.segments[i].position*=-1;
				rightTrajectory.segments[i].velocity*=-1;
				rightTrajectory.segments[i].acceleration*=-1;
				rightTrajectory.segments[i].jerk*=-1;
			}
		} else {
			leftTrajectory = modifier.getLeftTrajectory();
			rightTrajectory = modifier.getRightTrajectory();
		}
		// pathfinder bug swaps left/right trajectories
		leftFollower.setTrajectory(flipLeftRight ? leftTrajectory : rightTrajectory);
		rightFollower.setTrajectory(flipLeftRight ? rightTrajectory : leftTrajectory);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		positionErrorLeftPositive = 0;
    		positionErrorLeftNegative = 0;
    		positionErrorRightPositive = 0;
    		positionErrorRightNegative = 0;
    		yawErrorPositive = 0;
    		yawErrorNegative = 0;
    		if (!trajectoryLoaded) {
    			// Load from file
    			File file = new File("/home/lvuser/motionprofiles/" + filename + ".traj");
    			initTrajectory(Pathfinder.readFromFile(file));
    		}
    		if (RobotMap.tuningMode) {
    			initFollowers();
    		}
    		leftFollower.configurePIDVA(kP.get(), 0, kD.get(), kV.get(), kA.get());
    		rightFollower.configurePIDVA(kP.get(), 0, kD.get(), kV.get(), kA.get());
    		leftFollower.reset();
    		rightFollower.reset();
    		initialYaw = absHeading ? 0 : Robot.ahrs.getYaw(); // if absolute yaw, don't correct
    		// getHeading only updates when calling calculate so get it from the segment instead
    		initialProfileYaw = absHeading ? 0 : leftFollower.getSegment().heading;
    		initialPositionLeft = Robot.driveSubsystem.getDistanceLeft();
    		initialPositionRight = Robot.driveSubsystem.getDistanceRight();
    		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
    			Robot.driveSubsystem.switchGear(gear);
    		}
    		Robot.driveSubsystem.changeStatusRate(sensorFrameRate);
    		Robot.driveSubsystem.changeControlRate(controlFrameRate);
    }

    // Called repeatedly when this Command is scheduled to run
    @SuppressWarnings("unused")
	protected void execute() {
    		double l = leftFollower.calculate(Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft);
    		double r = rightFollower.calculate(Robot.driveSubsystem.getDistanceRight()-initialPositionRight);
    		
    		gyroHeading = Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw()-initialYaw);
        	desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading()-initialProfileYaw)); // Pathfinder native headings are in radians
        	desiredHeading*= flipLeftRight ? -1 : 1;
        	
        angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        	
        	// At end, load less aggressive parameters for final adjustment
        	if (!leftFollower.isFinished()) {
        		// in else to avoid measuring measure final adjust
        		double error;
        		error = leftFollower.getLastError();
        		if (error > 0) {
        			positionErrorLeftPositive += error;
        		} else if (error < 0) {
        			positionErrorLeftNegative += Math.abs(error);
        		}
        		error = rightFollower.getLastError();
        		if (error > 0) {
        			positionErrorRightPositive += error;
        		} else if (error < 0) {
        			positionErrorRightNegative += Math.abs(error);
        		}
        		error = angleDifference;
        		if (error > 0) {
        			yawErrorPositive += error;
        		} else if (error < 0) {
        			yawErrorNegative += Math.abs(error);
        		}
        	}
        	
        	double turn = 0;
        	// system so that if yaw is off, correct for that without oscillation of position control
        	if (leftFollower.isFinished()) {
        		turn = enableGyroCorrection ? kPAngleAdjust.get() * angleDifference : 0;
        		Robot.driveSubsystem.driveInchesPerSec(turn, turn*-1);
        	} else {
        		turn = enableGyroCorrection ? kPAngle.get() * angleDifference : 0;
        		Robot.driveSubsystem.driveInchesPerSec((l + turn), (r - turn));
        	}
        	
        	// Graphing
        	if (RobotMap.tuningMode) {
	        	SmartDashboard.putString("Profile Position Graph", Robot.genGraphStr(
	        			leftFollower.getSegment().position,
	        			Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft,
	        			rightFollower.getSegment().position,
	        			Robot.driveSubsystem.getDistanceRight()-initialPositionRight));
	        	SmartDashboard.putNumber("Profile Target Position Left", leftFollower.getSegment().position);
	        	SmartDashboard.putNumber("Profile Target Position Right", rightFollower.getSegment().position);
	        	SmartDashboard.putNumber("Profile Current Position Left", Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft);
	        	SmartDashboard.putNumber("Profile Current Position Right", Robot.driveSubsystem.getDistanceRight()-initialPositionRight);
	        	SmartDashboard.putString("Profile Heading Graph", Robot.genGraphStr(
	        			desiredHeading, gyroHeading));
	        	SmartDashboard.putNumber("Profile Target Heading", desiredHeading);
	        	SmartDashboard.putNumber("Profile Current Heading", gyroHeading);
	        	
	        	SmartDashboard.putNumber("Profile P output", kP.get()*((leftFollower.getLastError()+rightFollower.getLastError())/2));
	        	SmartDashboard.putNumber("Profile V output", kV.get()*((leftFollower.getSegment().velocity+rightFollower.getSegment().velocity)/2));
	        	SmartDashboard.putNumber("Profile A output", kA.get()*((leftFollower.getSegment().acceleration+rightFollower.getSegment().acceleration)/2));
	        	SmartDashboard.putNumber("Profile Error", (leftFollower.getLastError() + rightFollower.getLastError())/2);
        	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		boolean tilted = Math.abs(Robot.ahrs.getRoll()) >= tiltThreshold || Math.abs(Robot.ahrs.getPitch()) >= tiltThreshold;
    		// current segment and heading should be same on left and right, only check one
    		// At end of profile only correct angle
		return leftFollower.isFinished() && !(endConverge == ConvergenceMode.IF_FLAT && tilted) && (endConverge == ConvergenceMode.NEVER || !enableGyroCorrection ||
				(Math.abs(angleDifference))
				<=AngleErrorThreshold.get());
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.driveSubsystem.stop();
    		Robot.driveSubsystem.resetSensorRate();
    		Robot.driveSubsystem.resetControlRate();
    		if (RobotMap.tuningMode) {
    			System.out.printf("DLeft: %f, DRight: %f, Yaw: %f, Target Yaw: %f, Target DLeft: %f, Target DRight: %f\n",
    					Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft, Robot.driveSubsystem.getDistanceRight()-initialPositionRight, gyroHeading,
    					desiredHeading, leftFollower.getSegment().position, rightFollower.getSegment().position);
    			System.out.printf("Errors: Lp: %f, Ln: %f, Rp: %f, Rn: %f, Yp: %f, Yn: %f\n",
    					positionErrorLeftPositive/trajectoryLength, positionErrorLeftNegative/trajectoryLength,
    					positionErrorRightPositive/trajectoryLength, positionErrorRightNegative/trajectoryLength,
    					yawErrorPositive/trajectoryLength, yawErrorNegative/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Left Positive", positionErrorLeftPositive/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Left Negative", positionErrorLeftNegative/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Right Positive", positionErrorRightPositive/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Right Negative", positionErrorRightNegative/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Yaw Positive", yawErrorPositive/trajectoryLength);
    			SmartDashboard.putNumber("Profile Error Yaw Negative", yawErrorNegative/trajectoryLength);
    			SmartDashboard.putNumber("Profile Final Error Left", 
    					Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft-leftFollower.getSegment().position);
    			SmartDashboard.putNumber("Profile Final Error Right", 
    					Robot.driveSubsystem.getDistanceRight()-initialPositionRight-rightFollower.getSegment().position);
    			SmartDashboard.putNumber("Profile Final Error Yaw", gyroHeading-Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading())));
    		}
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
    
    public enum ConvergenceMode {
    		NEVER,
    		/**
    		 * Only finish if the robot is flat
    		 */
    		IF_FLAT,
    		ALWAYS
    }
    
    
    // based on DistanceFollower, adds getLastError, will track last point instead of returning 0
    private class CustomDistanceFollower {
        @SuppressWarnings("unused")
		private double kp, ki, kd, kv, ka;

        private double last_error, heading;

        private int segment;
        private Trajectory trajectory;

        @SuppressWarnings("unused")
		public CustomDistanceFollower(Trajectory traj) {
            this.trajectory = traj;
        }

        @SuppressWarnings("unused")
		public CustomDistanceFollower() { }

        /**
         * Set a new trajectory to follow, and reset the cumulative errors and segment counts
         */
        @SuppressWarnings("unused")
		public void setTrajectory(Trajectory traj) {
            this.trajectory = traj;
            reset();
        }

        /**
         * Configure the PID/VA Variables for the Follower
         * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
         * @param ki The integral term. Currently unused.
         * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
         * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
         *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
         *           motor controllers
         * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
         */
        public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
            this.kp = kp; this.ki = ki; this.kd = kd;
            this.kv = kv; this.ka = ka;
        }

        /**
         * Reset the follower to start again. Encoders must be reconfigured.
         */
        public void reset() {
            last_error = 0; segment = 0;
        }

        /**
         * Calculate the desired output for the motors, based on the distance the robot has covered.
         * This does not account for heading of the robot. To account for heading, add some extra terms in your control
         * loop for realignment based on gyroscope input and the desired heading given by this object.
         * @param distance_covered  The distance covered in meters
         * @return                  The desired output for your motor controller
         */
        // Modified to continue tracking last point at end of trajectory
        public double calculate(double distance_covered) {
        	Trajectory.Segment seg = trajectory.get(segment);
        	double error = seg.position - distance_covered;
        	double calculated_value =
        			kp * error +                                    // Proportional
        			kd * ((error - last_error) / seg.dt) +          // Derivative
        			(kv * seg.velocity + ka * seg.acceleration);    // V and A Terms
        	last_error = error;
        	heading = seg.heading;
        	if (segment < trajectory.length()-1) {
        		segment++;
        	}

        	return calculated_value;
        }

        /**
         * @return the desired heading of the current point in the trajectory
         */
        public double getHeading() {
            return heading;
        }

        /**
         * @return the current segment being operated on
         */
        @SuppressWarnings("unused")
		public Trajectory.Segment getSegment() {
            return trajectory.get(segment);
        }

        /**
         * @return the position error last time calculate was called 
         */
        public double getLastError() {
			return last_error;
		}

		/**
         * @return whether we have finished tracking this trajectory or not.
         */
        // modified to work with my modified calculate that will not advance segment beyond end (added -1)
        public boolean isFinished() {
            return segment >= trajectory.length()-1;
        }
    }
}
