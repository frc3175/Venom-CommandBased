// Used by FRC-3175 on 03/13/20
package frc.robot.utilities;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Wrapper class for all DoubleSolenoids
 * 
 * @author FRC-3539
 *
 * @since 11/12/17
 */
public class ByteDoubleSolenoid extends DoubleSolenoid
{
	private boolean isTrigger;

	/**
	 * Allows us to create a double solenoid object.
	 * 
	 * @param pcm The CAN id of the pcm.
	 * @param on The port that the on wire of the solenoid is plugged into.
	 * @param off The port that the off wire of the solenoid is plugged into.
	 * @param isTrigger The direction that the solenoid will default to.
	 */
	public ByteDoubleSolenoid(int pcm, int on, int off, boolean isTrigger)
	{
		super(pcm, on, off);
		setPosition(isTrigger);
	}

	/**
	 * Allows us to toggle a double solenoid object so that it is the opposite
	 * direction that it currently is in however if the solenoid is off it will
	 * return to the default position.
	 */
	public void toggle() {
		if (get() == DoubleSolenoid.Value.kOff)
		{
			setPosition(isTrigger);
		}
		else if (get() == DoubleSolenoid.Value.kReverse)
		{
			forward();
		}
		else if (get() == DoubleSolenoid.Value.kForward)
		{
			reverse();
		}
	}

	/**
	 * Allows us to set the position of the double solenoid object.
	 * 
	 * @param shouldTrigger The position that the solenoid will move to.
	 */
	public void setPosition(boolean shouldTrigger) {
		if (shouldTrigger)
			forward();
		else
			reverse();
	}

	/**
	 * Allows us to set the solenoid to the forward position.
	 */
	public void forward() {
		set(DoubleSolenoid.Value.kForward);
	}

	/**
	 * Allows us to set the solenoid to the reverse position.
	 */
	public void reverse() {
		set(DoubleSolenoid.Value.kReverse);
	}

	/**
	 * Allows us to set the solenoid to the disabled position.
	 */
	public void disable() {
		set(DoubleSolenoid.Value.kOff);
	}

	public boolean isTrigger() { return isTrigger; }
}