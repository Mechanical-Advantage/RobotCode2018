package org.usfirst.frc.team6328.robot.triggers;

import java.util.Arrays;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * A trigger that requires multiple buttons to be pressed
 */
public class MultiButtonTrigger extends Trigger {
	
	JoystickButton[] buttons;
	
	public MultiButtonTrigger(JoystickButton...buttons) {
		this.buttons = buttons;
	}

	public boolean get() {
		// Use a stream to check if all buttons match a condition
		return Arrays.stream(buttons).allMatch(button -> button.get());
	}
}
