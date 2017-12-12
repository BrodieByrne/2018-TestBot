package org.usfirst.frc.team88.robot;

import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc.team88.robot.commands.Shift;
import org.usfirst.frc.team88.robot.commands.ZeroEncoders;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static final int LEFT_HORIZ_AXIS = 0;
	private static final int LEFT_VERT_AXIS = 1;
	private static final int RIGHT_HORIZ_AXIS = 4;
	private static final int RIGHT_VERT_AXIS = 5;
	private static final int LEFT_Z_AXIS = 3;
	private static final int RIGHT_Z_AXIS = 2;

	private static final double STICK_DEADZONE = 0.3;
	private static final double STICK_MAX = 0.97;

	// driver controller setup
	private Joystick driverController = new Joystick(0);
	private Button driverButtonA = new JoystickButton(driverController, 1);
	private Button driverButtonB = new JoystickButton(driverController, 2);
	private Button driverButtonX = new JoystickButton(driverController, 3);
	private Button driverButtonY = new JoystickButton(driverController, 4);
	private Button driverButtonLeftBumper = new JoystickButton(driverController, 5);
	private Button driverButtonRightBumper = new JoystickButton(driverController, 6);
	private Button driverButtonBack = new JoystickButton(driverController, 7);
	private Button driverButtonStart = new JoystickButton(driverController, 8);
	private Button driverButtonLeftAxisPress = new JoystickButton(driverController, 9);
	private Button driverButtonRightAxisPress = new JoystickButton(driverController, 10);

	// operator controller setup
	private Joystick operatorController = new Joystick(1);
	private Button operatorButtonA = new JoystickButton(operatorController, 1);
	private Button operatorButtonB = new JoystickButton(operatorController, 2);
	private Button operatorButtonX = new JoystickButton(operatorController, 3);
	private Button operatorButtonY = new JoystickButton(operatorController, 4);
	private Button operatorButtonLeftBumper = new JoystickButton(operatorController, 5);
	private Button operatorButtonRightBumper = new JoystickButton(operatorController, 6);
	private Button operatorButtonBack = new JoystickButton(operatorController, 7);
	private Button operatorButtonStart = new JoystickButton(operatorController, 8);
	private Button operatorButtonLeftAxisPress = new JoystickButton(operatorController, 9);
	private Button operatorButtonRightAxisPress = new JoystickButton(operatorController, 10);

	public OI() {
		//// TRIGGERING COMMANDS WITH BUTTONS
		// Once you have a button, it's trivial to bind it to a button in one of
		// three ways:

		// button.whenPressed(new ExampleCommand());
		//    Start the command when the button is pressed and let it run the command
		//    until it is finished as determined by it's isFinished method.

		// button.whileHeld(new ExampleCommand());
		//    Run the command while the button is being held down and interrupt it once
		//    the button is released.

		// button.whenReleased(new ExampleCommand());
		//    Start the command when the button is released and let it run the command
		//    until it is finished as determined by it's isFinished method.

		driverButtonRightBumper.whenPressed(new Shift());
		driverButtonLeftBumper.whenPressed(new ZeroEncoders());
		

	}


	// Utility functions


	// driver controller

	public double getDriverRightY() {
		return driverController.getRawAxis(RIGHT_VERT_AXIS);
	}

	public double getDriverRightX() {
		return driverController.getRawAxis(RIGHT_HORIZ_AXIS);
	}

	public double getDriverLeftY() {
		return driverController.getRawAxis(LEFT_VERT_AXIS);
	}

	public double getDriverLeftX() {
		return driverController.getRawAxis(LEFT_HORIZ_AXIS);
	}

	public double getDriverLeftTrigger() {
		return driverController.getRawAxis(LEFT_Z_AXIS);
	}

	public double getDriverRightTrigger() {
		return driverController.getRawAxis(RIGHT_Z_AXIS);
	}

	public double getDriverZ() {
		return driverController.getRawAxis(LEFT_Z_AXIS) - driverController.getRawAxis(RIGHT_Z_AXIS);
	}

	public void rumbleDriver(double rumble) {
		driverController.setRumble(RumbleType.kLeftRumble, rumble);
		driverController.setRumble(RumbleType.kRightRumble, rumble);
	}

	public void rumbleDriverLeft(double rumble) {
		driverController.setRumble(RumbleType.kLeftRumble, rumble);
	}

	public void rumbleDriverRight(double rumble) {
		driverController.setRumble(RumbleType.kRightRumble, rumble);
	}



	// operator controller

	public double getOperatorRightY() {
		return -operatorController.getRawAxis(RIGHT_VERT_AXIS);
	}

	public double getOperatorRightX() {
		return operatorController.getRawAxis(RIGHT_HORIZ_AXIS);
	}

	public double getOperatorLeftY() {
		return -operatorController.getRawAxis(LEFT_VERT_AXIS);
	}

	public double getOperatorLeftX() {
		return operatorController.getRawAxis(LEFT_HORIZ_AXIS);
	}

	public double getOperatorLeftTrigger() {
		return operatorController.getRawAxis(LEFT_Z_AXIS);
	}

	public double getOperatorRightTrigger() {
		return operatorController.getRawAxis(RIGHT_Z_AXIS);
	}

	public double getOperatorZ() {
		return operatorController.getRawAxis(LEFT_Z_AXIS) - operatorController.getRawAxis(RIGHT_Z_AXIS);
	}

	public void rumbleOperator(double rumble) {
		operatorController.setRumble(RumbleType.kLeftRumble, rumble);
		operatorController.setRumble(RumbleType.kRightRumble, rumble);
	}

	public void rumbleOperatorLeft(double rumble) {
		operatorController.setRumble(RumbleType.kLeftRumble, rumble);
	}

	public void rumbleOperatorRight(double rumble) {
		operatorController.setRumble(RumbleType.kRightRumble, rumble);
	}



	// Miscellaneous utilities

	public double applyDeadZone(double value) {
		if (Math.abs(value) < STICK_DEADZONE) {
			return 0.0;
		} else if (value > 0) {
			value = (value - STICK_DEADZONE) / (1 - STICK_DEADZONE);
		} else {
			value = (value + STICK_DEADZONE) / (1 - STICK_DEADZONE);
		}

		return value;
	}

	public double applyMaxValue(double value){
		if(value >= STICK_MAX){
			return 1.0;
		}
		else if (value <= -STICK_MAX){
			return -1.0;
		}
		else{
			return value;
		}
	}

	public double applySquare(double value) {
		double output;
		output = (value == 0.0 ? 0.0 : Math.pow(value, 3)/Math.abs(value));
		return (Math.abs(output) < 0.05 ? 0.0 : output);
	}
}



