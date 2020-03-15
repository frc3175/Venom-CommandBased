/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * A place to store PID gains and feedforward gains
 */
public class Gains {

  public static class shooterPID {
    public static double kP = 0.0015;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.0005;
    public static int kSlotIDx = 0;
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
  }

  // Turn to angle PID
  public static class TurnToAngleGains {
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
  }

  // Turret PID
  public static class TurretPID {
    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 5;
  }

  // Drive straight PID
  public static class DriveStabilization {
    public static double kStabilizationP = 1;
    public static double kStabilizationI = 0.5;
    public static double kStabilizationD = 0;
  }

}