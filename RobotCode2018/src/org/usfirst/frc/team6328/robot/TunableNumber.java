package org.usfirst.frc.team6328.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or value not in dashboard.
 * @author elliot
 *
 */
public class TunableNumber {
	private String key;
	private double defaultValue;
	
	/**
	 * Create a new TunableNumber
	 * @param dashboardKey Key on dashboard
	 */
	public TunableNumber(String dashboardKey) {
		this.key = dashboardKey;
	}

	/**
	 * Get the default value for the number that has been set
	 * @return The default value
	 */
	public double getDefault() {
		return defaultValue;
	}

	/**
	 * Set the default value of the number
	 * @param defaultValue The default value
	 */
	public void setDefault(double defaultValue) {
		this.defaultValue = defaultValue;
	}
	
	/**
	 * Get the current value, from dashboard if available and in tuning mode
	 * @return The current value
	 */
	public double get() {
		return RobotMap.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
	}
}