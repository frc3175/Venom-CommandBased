package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * @author FRC-3175 - Ian Sjögren
 *
 * @since 03/13/20
 *
 * @version 1.01 (Post-Infinite-Recharge)
 */

public class DriveSubsystem extends SubsystemBase {
    private static WPI_TalonFX rightMotorFront, rightMotorBack, leftMotorFront, leftMotorBack;
    private static AHRS gyro;
    private static double m_quickStopAccumulator;

    // Constructor that is run once
    public DriveSubsystem() {

        // Drive train motors
        rightMotorFront = new WPI_TalonFX(RobotContainer.robotConstants.getDriveConstants().getDT_TALON_LEFTFRONT());
        rightMotorBack = new WPI_TalonFX(RobotContainer.robotConstants.getDriveConstants().getDT_TALON_RIGHTBACK());

        leftMotorFront = new WPI_TalonFX(RobotContainer.robotConstants.getDriveConstants().getDT_TALON_RIGHTFRONT());
        leftMotorBack = new WPI_TalonFX(RobotContainer.robotConstants.getDriveConstants().getDT_TALON_RIGHTBACK());

        gyro = new AHRS(SPI.Port.kMXP);

        rightMotorBack.follow(rightMotorFront);
        leftMotorBack.follow(leftMotorFront);

    }

    /**
     * @return Resets gyro Yaw and Angle
     */
    public static void resetGyro() {
        gyro.reset();
    }

    // Tank Drive used for limelight lineup
    public static void drive(double powerLeft, double powerRight) {
        leftMotorFront.set(ControlMode.PercentOutput, powerLeft);
        rightMotorFront.set(ControlMode.PercentOutput, powerRight);
    }

    /**
     *
     * @param xSpeed       The robot's speed along the X axis [-1.0..1.0]. Forward
     *                     is positive.
     * @param zRotation    The robot's rotation rate around the Z axis [-1.0..1.0].
     *                     Clockwise is positive.
     * @param squareInputs If set, decreases the input sensitivity at low speeds.
     */
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        leftMotorFront.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * 1);
        double maxOutput = 1 * -1;
        rightMotorFront.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);
    }

    /**
     * @return gyro is connected
     */
    public static boolean isConnected() {
        return gyro.isConnected();
    }

    /**
     *
     * @param xSpeed      The robot's speed along the X axis [-1.0..1.0]. Forward is
     *                    positive.
     * @param zRotation   The robot's rotation rate around the Z axis [-1.0..1.0].
     *                    Clockwise is positive.
     * @param isQuickTurn If set, overrides constant-curvature turning for
     *                    turn-in-place maneuvers.
     */
    @SuppressWarnings({ "ParameterName", "PMD.CyclomaticComplexity" })
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < RobotContainer.robotConstants.getDriveConstants().getKDefaultQuickStopThreshold()) {
                m_quickStopAccumulator = (1
                        - RobotContainer.robotConstants.getDriveConstants().getkDefaultQuickStopAlpha())
                        * m_quickStopAccumulator
                        + RobotContainer.robotConstants.getDriveConstants().getkDefaultQuickStopAlpha()
                                * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation * 0.7;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        leftMotorFront.set(ControlMode.PercentOutput, leftMotorOutput * 1);
        rightMotorFront.set(ControlMode.PercentOutput, rightMotorOutput * 1 * -1);
    }

    public static double getEncoderDistanceRight() {
        return rightMotorFront.getSelectedSensorPosition();
    }

    public static double getEncoderDistanceLeft() {
        return leftMotorFront.getSelectedSensorPosition();
    }

    public static double getVelocityRight() {
        return rightMotorFront.getSelectedSensorVelocity();
    }

    public static double getVelocityLeft() {
        return leftMotorFront.getSelectedSensorVelocity();
    }

    public static double getAvgVelocity() {
        return (leftMotorFront.getSelectedSensorVelocity());
    }

    public static double getEncoderAverage() {
        return (Math.abs(getEncoderDistanceLeft() + getEncoderDistanceRight()) / 2);
    }

    public static void resetEncoder() {
        leftMotorFront.setSelectedSensorPosition(0, 0, 10);
        rightMotorFront.setSelectedSensorPosition(0, 0, 10);
    }

    /**
     * 
     * @return gyro angle
     */
    public static double getAngle() {
        return gyro.getAngle();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (RobotContainer.robotConstants.getDriveConstants().getGyroReversed() ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (RobotContainer.robotConstants.getDriveConstants().getGyroReversed() ? -1.0 : 1.0);
    }

    // Diagnostics Used in the diagnostic subsystem
    public static double getRightMotorFrontTemp() {
        return rightMotorFront.getTemperature();
    }

    public static double getRightMotorBackTemp() {
        return rightMotorBack.getTemperature();
    }

    public static double getLeftMotorFrontTemp() {
        return leftMotorFront.getTemperature();
    }

    public static double getLeftMotorBackTemp() {
        return leftMotorBack.getTemperature();
    }

    public static double getRightMotorFrontCurrent() {
        return rightMotorFront.getSupplyCurrent();
    }

    public static double getRightMotorBackCurrent() {
        return rightMotorBack.getSupplyCurrent();
    }

    public static double getLeftMotorFrontCurrent() {
        return leftMotorFront.getSupplyCurrent();
    }

    public static double getLeftMotorBackCurrent() {
        return leftMotorBack.getSupplyCurrent();
    }

}