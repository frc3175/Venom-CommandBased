/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * An interface conta
 */
public abstract class Constants
{

    public abstract RobotIDConstants getRobotIDConstants();

    public abstract DriveConstants getDriveConstants();

    public abstract OIConstants getOIConstants();

    public abstract AutoConstants getAutoConstants();

    public abstract IntakeConstants getIntakeConstants();

    public abstract ClimbConstants getClimbConstants();

    public abstract ShooterConstants getShooterConstants();

    public abstract LimelightConstants getLimelightConstants();

    public abstract PIDConstants getPIDConstants();

    public abstract class RobotIDConstants
    {
        public abstract int getIntakeMotorID();

        public abstract int getTopShooterMotorID();

        public abstract int getBottomShooterMotorID();

        public abstract int getShooterRotationMotorID();

        public abstract int getHoodedAngleMotorID();

        public abstract int getHopperMotorID();

        public abstract int getRotationMotorID();

        public abstract int getClimbMotorLeftTalon();

        public abstract int getClimbMotorRightTalon();

        public abstract int getFolderLeftVictor();

        public abstract int getFolderRightVictor();

        public abstract int getPCMID();

        public abstract int getIntakeSolenoidOn();

        public abstract int getIntakeSolenoidOff();
    }

    public abstract class DriveConstants
    {
        public abstract int getDT_TALON_LEFTFRONT();

        public abstract int getDT_TALON_LEFTBACK();

        public abstract int getDT_TALON_RIGHTFRONT();

        public abstract int getDT_TALON_RIGHTBACK();

        public abstract double getKDefaultQuickStopThreshold();

        public abstract double getkDefaultQuickStopAlpha();

        public abstract double getDriveTrainDeadband();

        public abstract double getTurnDeadband();

        public abstract double getDriveStraightConstant();

    }

    public abstract class OIConstants
    {
        public abstract int getKDriverControllerPort();

        public abstract int getKOpControllerPort();

        public abstract int getKClimberControllerPort();
    }

    public abstract class AutoConstants
    {
 

    }

    public abstract class IntakeConstants
    {

        public abstract boolean getIntakeMotorInverted();

        public abstract boolean getDefaultIntakeDirection();

        public abstract double getIntakePercentOutput();

        public abstract double getHopperSpeedForward();

        public abstract double getHopperSpeedReverse();

    }

    public abstract class ClimbConstants {

        public abstract double getFoldSetSpeed();

        public abstract int getUpPosition();

        public abstract int getDownPosition();
    }

    public abstract class ShooterConstants {
    }

    public abstract class LimelightConstants
    {
        public abstract double getCameraAngle();

        public abstract double getCameraHeight();

        public abstract double getPowerPortHeight();

        public abstract double getLimelightOffset();
    }

    public abstract class PIDConstants {

        public abstract int getkSlotIDx();

        public abstract int getkPIDLoopIdx();

        public abstract int getkTimeoutMs();

        public abstract double getkP();

        public abstract double getkI();

        public abstract double getkD();

        public abstract double getkF();
    
    }
}
