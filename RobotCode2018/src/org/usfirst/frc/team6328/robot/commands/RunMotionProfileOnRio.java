package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Runs a motion profile on the roboRIO. dt of profile must be 0.02.
 */
public class RunMotionProfileOnRio extends Command {
	
	private final int sensorFrameRate = 2; // ms
	// Note: Control frame rate must be defined in CANTalon constructor, so that can't be set
	private final boolean enableGyroCorrection = false; // false for tuning
	
	private double kP;
	private double kD;
	private double kPAdjust;
	private double kDAdjust;
	private double PositionErrorThreshold;
	private double AngleErrorThreshold;
	private double wheelbase;
	private double kPAngle; // standard is 0.8 * (1.0/80.0), add max velocity like: 0.8*60 * (1.0/80.0) (if maxVel is 60)
	// kPAngle should be positive
	
	private CustomDistanceFollower leftFollower, rightFollower;
	private double initialYaw;
	private double initialPositionLeft, initialPositionRight;
	private double gyroHeading;
	private double positionErrorLeftPositive, positionErrorLeftNegative, positionErrorRightPositive,
		positionErrorRightNegative, yawErrorPositive, yawErrorNegative;
	private int trajectoryLength;
	
	/*
     * Tuning Notes:
     * High P to keep up with trajectory
     * Tune wheelbase with gyro correction disabled until heading is about accurate
     * D should be significantly lower than P
     * Too much D will overcorrect, robot will go too fast, not much is needed, in my initial test, any causes problems
     */

    public RunMotionProfileOnRio(Trajectory trajectory) {
    		super("RunMotionProfileOnRio");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.driveSubsystem);
    		if (RobotMap.robot == RobotType.PRACTICE) {
    			// not tuned
    		} else {
    			// tuned using max velocity: 100, accel: 55, jerk: 2700
    			kP = 80;
    			kD = 0;
    			kPAdjust = 3;
    			kDAdjust = 0;
    			PositionErrorThreshold = 0.75;
    			AngleErrorThreshold = 1;
    			wheelbase = 21; // 18 measured
    			kPAngle = 0.8*60 * (1.0/80.0);
    		}
    		trajectoryLength = trajectory.length();
    		TankModifier modifier = new TankModifier(trajectory);
    		modifier.modify(wheelbase);
    		// pathfinder bug swaps left/right trajectories
    		leftFollower = new CustomDistanceFollower(modifier.getRightTrajectory());
    		rightFollower = new CustomDistanceFollower(modifier.getLeftTrajectory());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		positionErrorLeftPositive = 0;
    		positionErrorLeftNegative = 0;
    		positionErrorRightPositive = 0;
    		positionErrorRightNegative = 0;
    		yawErrorPositive = 0;
    		yawErrorNegative = 0;
    		leftFollower.configurePIDVA(kP, 0, kD, 1, 0);
    		rightFollower.configurePIDVA(kP, 0, kD, 1, 0);
    		leftFollower.reset();
    		rightFollower.reset();
    		initialYaw = Robot.ahrs.getYaw();
    		initialPositionLeft = Robot.driveSubsystem.getDistanceLeft();
    		initialPositionRight = Robot.driveSubsystem.getDistanceRight();
    		Robot.driveSubsystem.changeStatusRate(sensorFrameRate);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		double l = leftFollower.calculate(Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft);
    		double r = rightFollower.calculate(Robot.driveSubsystem.getDistanceRight()-initialPositionRight);
    		
    		gyroHeading = Pathfinder.boundHalfDegrees(Robot.ahrs.getYaw()-initialYaw);
        	double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading())); // Pathfinder native headings are in radians
        	
        	double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        	double turn = 0;
        	if (enableGyroCorrection) {
        		turn = kPAngle * angleDifference;
        	}
        	        	
        	// At end, load less aggressive parameters for final adjustment
        	if (leftFollower.isFinished()) {
        		// set velocity to 0 so last profile velocity is not still applied, in case last velocity is not zero
        		leftFollower.configurePIDVA(kPAdjust, 0, kDAdjust, 0, 0);
        		rightFollower.configurePIDVA(kPAdjust, 0, kDAdjust, 0, 0);
        	} else {
        		// don't measure final adjust
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
        	
        	// system so that if yaw is off, correct for that without oscillation of position control
        	if (leftFollower.isFinished() && leftFollower.getLastError()<=PositionErrorThreshold &&
        				rightFollower.getLastError()<=PositionErrorThreshold && enableGyroCorrection) {
        		Robot.driveSubsystem.driveInchesPerSec(turn, turn*-1);
        	} else {
        		Robot.driveSubsystem.driveInchesPerSec((r - turn), (l + turn));
        	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		// current segment and heading should be same on left and right, only check one
		return leftFollower.isFinished() && leftFollower.getLastError()<=PositionErrorThreshold &&
				rightFollower.getLastError()<=PositionErrorThreshold &&
				Math.abs(gyroHeading-Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading())))<=AngleErrorThreshold;
    }

    // Called once after isFinished returns true
    protected void end() {
    		Robot.driveSubsystem.stop();
    		Robot.driveSubsystem.resetSensorRate();
    		System.out.printf("DLeft: %f, DRight: %f, Yaw: %f, Target Yaw: %f, Target DLeft: %f, Target DRight: %f\n",
			Robot.driveSubsystem.getDistanceLeft()-initialPositionLeft, Robot.driveSubsystem.getDistanceRight()-initialPositionRight, gyroHeading,
			Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading())), leftFollower.getSegment().position, rightFollower.getSegment().position);
    		System.out.printf("Errors: Lp: %f, Ln: %f, Rp: %f, Rn: %f, Yp: %f, Yn: %f\n",
    				positionErrorLeftPositive/trajectoryLength, positionErrorLeftNegative/trajectoryLength,
    				positionErrorRightPositive/trajectoryLength, positionErrorRightNegative/trajectoryLength,
    				yawErrorPositive, yawErrorNegative);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
    
    
    // based on DistanceFollower, adds getLastError, will track last point instead of returning 0
    private class CustomDistanceFollower {
        @SuppressWarnings("unused")
		private double kp, ki, kd, kv, ka;

        private double last_error, heading;

        private int segment;
        private Trajectory trajectory;

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
