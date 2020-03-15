package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utilities.DirectionalButton;
import frc.robot.utilities.TriggerButton;
import frc.robot.utilities.DirectionalButton.Direction;

public class XBoxController extends Joystick{
	
	// Button numbers each object contains a number that directly references a button.
	private static final int A = 1;
	private static final int B = 2;
	private static final int X = 3;
	private static final int Y = 4;

	private static final int START = 8;
	private static final int SELECT = 7;

	private static final int LS = 9;
	private static final int RS = 10;

	private static final int BL = 5;
	private static final int BR = 6;

	private static final int RIGHT_TRIGGER = 3;
	private static final int LEFT_TRIGGER = 2;

	// Axis numbers each object contains a number that directly references an axis.
	private static final int X_AxisL = 0;
	private static final int Y_AxisL = 1;

	private static final int X_AxisR = 4;
	private static final int Y_AxisR = 5;

	/**
	 * Constructor of the XBoxController wrapper requires a port number
	 * between 0-5.
	 * 
	 * @param port the usb port number that the controller is plugged into(this can
	 *            be obtained through the driver station).
	 */
	public XBoxController(int port) {
		super(port);
	}
    
    /**
     * Haptic feedback on left side of the controller
     * 
     * @param rumble Value 0 to 1
     */
	public void setLeftRumble(double rumble) {
		setRumble(Joystick.RumbleType.kLeftRumble, (float)rumble);
	}
    
    /**
     * Haptic feedback on right side of the controller
     * 
     * @param rumble Value 0 to 1
     */
	public void setRightRumble(double rumble) {
		setRumble(Joystick.RumbleType.kRightRumble, (float)rumble);
    }
    
    /**
     * Haptic feedback on both sides of the controller
     * 
     * @param rumble Value 0 to 1
     */
	public void setDoubleRumble(double rumble) {
		setRumble(Joystick.RumbleType.kLeftRumble, (float)rumble);
		setRumble(Joystick.RumbleType.kRightRumble, (float)rumble);
	}

	/**
	 * refers to the <b>X</b> button on the controller
	 */
	public JoystickButton buttonX = new JoystickButton(this, X);
	/**
	 * refers to the <b>Y</b> button on the controller
	 */
	public JoystickButton buttonY = new JoystickButton(this, Y);
	/**
	 * refers to the <b>A</b> button on the controller
	 */
	public JoystickButton buttonA = new JoystickButton(this, A);
	/**
	 * refers to the <b>B</b> button on the controller
	 */
	public JoystickButton buttonB = new JoystickButton(this, B);

	/**
	 * refers to the <b>Left Bumper</b> button on the controller
	 */
	public JoystickButton buttonLeftBumper = new JoystickButton(this, BL);
	/**
	 * refers to the <b>Right Bumper</b> button on the controller
	 */
    public JoystickButton buttonRightBumper = new JoystickButton(this, BR);
    
    /**
     * 
     * @return if left bumper has been pressed
     */
    public boolean getLeftBumper() {
		return getRawButton(5);
	}

    /**
     * @return if right rumper has been pressed
     */
	public boolean getRightBumper() {
		return getRawButton(6);
	}

	/**
     * @return if start button has been pressed
     */
	public boolean getStartButton() {
		return getRawButton(8);
	}


	public TriggerButton buttonLeftTrigger = new TriggerButton(this, LEFT_TRIGGER);
	public TriggerButton buttonRightTrigger = new TriggerButton(this, RIGHT_TRIGGER);
	// public TriggerButton buttonRX = new TriggerButton(this,X_AxisR);

	/**
	 * refers to the <b>Start</b> button on the controller
	 */
	public JoystickButton buttonSTART = new JoystickButton(this, START);
	/**
	 * refers to the <b>Select</b> button on the controller
	 */
	public JoystickButton buttonSELECT = new JoystickButton(this, SELECT);

	/**
	 * refers to the <b>Left Stick</b> button on the controller
	 */
	public JoystickButton buttonLeftStick = new JoystickButton(this, LS);
	/**
	 * refers to the <b>Right Stick</b> button on the controller
	 */
	public JoystickButton buttonRightStick = new JoystickButton(this, RS);

	/**
	 * Returns the left trigger value this value should be between 0 and 1;
	 * 
	 * @return the left trigger value between 0 and 1.
	 */
	public double getLeftTrigger() { return getRawAxis(LEFT_TRIGGER); }

	/**
	 * Returns the right trigger value this value should be between 0 and 1;
	 * 
	 * @return the right trigger value between 0 and 1.
	 */
	public double getRightTrigger() { return getRawAxis(RIGHT_TRIGGER); }

	/**
	 * Returns the left stick x axis value this value should be between -1 and 1;
	 * 
	 * @return the left stick x axis value between -1 and 1.
	 */
	public double getLeftStickX() { return getRawAxis(X_AxisL); }

	/**
	 * Returns the left stick y axis value this value should be between -1 and 1;
	 * 
	 * @return the left stick y axis value between -1 and 1.
	 */
	public double getLeftStickY() { return -getRawAxis(Y_AxisL); }

	/**
	 * Returns the right stick x axis value this value should be between -1 and 1;
	 * 
	 * @return the right stick x axis value between -1 and 1.
	 */
	public double getRightStickX() { return getRawAxis(X_AxisR); }

	/**
	 * Returns the right stick y axis value this value should be between -1 and 1;
	 * 
	 * @return the right stick y axis value between -1 and 1.
	 */
	public double getRightStickY() {
		// Negative due to left being right and right being left...
		return -getRawAxis(Y_AxisR);
    }
    
	/**
	 * refers to the <b>Left Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadLeft = new DirectionalButton(this, Direction.LEFT);
	/**
	 * refers to the <b>Up Left Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadUpLeft = new DirectionalButton(this, Direction.UPLEFT);
	/**
	 * refers to the <b>Down Left Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadDownLeft = new DirectionalButton(this, Direction.DOWNLEFT);
	/**
	 * refers to the <b>Right Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadRight = new DirectionalButton(this, Direction.RIGHT);
	/**
	 * refers to the <b>Up Right Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadUpRight = new DirectionalButton(this, Direction.UPRIGHT);
	/**
	 * refers to the <b>Down Right Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadDownRight = new DirectionalButton(this, Direction.DOWNRIGHT);
	/**
	 * refers to the <b>Top Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadUp = new DirectionalButton(this, Direction.UP);
	/**
	 * refers to the <b>Bottom Directional Pad</b> button on the controller
	 */
	public DirectionalButton buttonPadDown = new DirectionalButton(this, Direction.DOWN);

}