/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.utilities.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class CompConstants extends Constants {

    public class RobotIDConstants extends Constants.RobotIDConstants {
        @Override
        public int getIntakeMotorID() {
            return 9;
        }

        @Override
        public int getTopShooterMotorID() {
            return 14;
        }

        @Override
        public int getBottomShooterMotorID() {
            return 15;
        }

        @Override
        public int getClimbMotorLeftTalon() {
            return 5;
        }

        @Override
        public int getClimbMotorRightTalon() {
            return 6;
        }

        @Override
        public int getPCMID() {
            return 0;
        }

        @Override
        public int getIntakeSolenoidOn() {
            return 0;
        }

        @Override
        public int getIntakeSolenoidOff() {
            return 1;
        }

        @Override
        public int getShooterRotationMotorID() {
            return 0;
        }

        @Override
        public int getHoodedAngleMotorID() {
            return 0; //PWM Port 0
        }

        @Override
        public int getHopperMotorID() {
            return 16;
        }

        @Override
        public int getFolderLeftVictor() {
            return 2;
        }

        @Override
        public int getFolderRightVictor() {
            return 3;
        }

        @Override
        public int getRotationMotorID() {
            return 4;
        }

    }

    public class DriveConstants extends Constants.DriveConstants {

        @Override
        public int getDT_TALON_LEFTFRONT() {
            return 10;
        }

        @Override
        public int getDT_TALON_LEFTBACK() {
            return 11;
        }

        @Override
        public int getDT_TALON_RIGHTFRONT() {
            return 12;
        }

        @Override
        public int getDT_TALON_RIGHTBACK() {
            return 13;
        }

        @Override
        public double getKDefaultQuickStopThreshold() {
            return 0.2;
        }

        @Override
        public double getkDefaultQuickStopAlpha() {
            return 0.1;
        }

        @Override
        public double getDriveTrainDeadband() {
            return 0.2;
        }

        @Override
        public double getTurnDeadband() {
            return 0.1;
        }

        @Override
        public double getDriveStraightConstant() {
            return 1;
        }

        @Override
        public double getTurnToleranceDeg() {
            return 5;
        }

        @Override
        public double getTurnRateToleranceDegPerS() {
            return 10;
        }

        @Override
        public boolean getGyroReversed() {
            return false;
        }
    }

    public class OIConstants extends Constants.OIConstants {
        public int getKDriverControllerPort() {
            return 0;
        }

        public int getKOpControllerPort() {
            return 1;
        }

        @Override
        public int getKClimberControllerPort() {
            return 2;
        }
    }

    public class AutoConstants extends Constants.AutoConstants {
    }

    public class IntakeConstants extends Constants.IntakeConstants {

        @Override
        public boolean getIntakeMotorInverted() {
            return false;
        }

        @Override
        public boolean getDefaultIntakeDirection() {
            return false;
        }

        @Override
        public double getIntakePercentOutput() {
            return -0.8;
        }

        @Override
        public double getHopperSpeedForward() {
            return 0.6;
        }

        @Override
        public double getHopperSpeedReverse() {
            return -0.6;
        }
    }

    public class ClimbConstants extends Constants.ClimbConstants {

        @Override
        public double getFoldSetSpeed() {
            return 1;
        }

        @Override
        public int getUpPosition() {
            return 90000;
        }

        @Override
        public int getDownPosition() {
            return 225000;
        }


    }

    public class ShooterConstants extends Constants.ShooterConstants {

    }
    public class LimelightConstants extends Constants.LimelightConstants {

        @Override
        public double getCameraAngle() {
            return 22.4; //degrees
        }

        @Override
        public double getCameraHeight() {
            return 23; //inches
        }

        /**
         * @return Returns inches
         */
        @Override
        public double getPowerPortHeight() {
            return 88; 
        }

        @Override
        public double getLimelightOffset() {
            return 0;
        }
    }

    private RobotIDConstants robotIDConstants = new RobotIDConstants();

    private DriveConstants robotDriveConstants = new DriveConstants();

    private OIConstants robotOIConstants = new OIConstants();

    private AutoConstants robotAutoConstants = new AutoConstants();

    private IntakeConstants robotIntakeConstants = new IntakeConstants();

    private ClimbConstants robotClimberConstants = new ClimbConstants();

    private ShooterConstants robotShooterConstants = new ShooterConstants();

    private LimelightConstants limelightConstants = new LimelightConstants();

    @Override
    public frc.robot.utilities.Constants.RobotIDConstants getRobotIDConstants() {
        return robotIDConstants;
    }

    @Override
    public frc.robot.utilities.Constants.DriveConstants getDriveConstants() {
        return robotDriveConstants;
    }

    @Override
    public frc.robot.utilities.Constants.OIConstants getOIConstants() {
        return robotOIConstants;
    }
    
    @Override
    public frc.robot.utilities.Constants.AutoConstants getAutoConstants() {
        return robotAutoConstants;
    }

    @Override
    public frc.robot.utilities.Constants.ShooterConstants getShooterConstants() {
        return robotShooterConstants;
    }

    @Override
    public IntakeConstants getIntakeConstants() {
        return robotIntakeConstants;
    }

    @Override
    public ClimbConstants getClimbConstants() {
        return robotClimberConstants;
    }

    @Override
    public LimelightConstants getLimelightConstants() {
        return limelightConstants;
    }

}