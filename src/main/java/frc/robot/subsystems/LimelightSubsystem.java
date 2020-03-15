package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Utils;

@SuppressWarnings("unused")
public class LimelightSubsystem extends SubsystemBase {
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static DriverStation ds = DriverStation.getInstance();

    private static boolean m_LimelightHasValidTarget = false;
    private static double m_LimelightDriveCommand = 0.0;
    private static double m_LimelightSteerCommand = 0.0;
    private static NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry camMode = table.getEntry("camMode");

    public static PIDController limelightPID = new PIDController(0.08, 0, 0);

    private static double limelightY = ty.getDouble(0.0);

    private static double[] distances = { 87, 150 }; // should be in Inches???
    public static double[] RPMs = { 5700, 5580 };

    public static void testFeed() {
        double x = table.getEntry("tx").getDouble(0.0);
        double y = table.getEntry("ty").getDouble(0.0);
    }

    public void forceLEDsOn() {
        table.getEntry("ledMode").forceSetValue(3);
    }

    /**
     * Sets limelight to vision processing mode
     */
    public void visionMode() {
        camMode.setNumber(0); // sets camera to vision processing mode
    }

    /**
     * Disables vision processing mode
     */
    public void driverMode() {
        camMode.setNumber(1); // sets camera to driving mode
    }

    public static void forceLEDsOff() {
        table.getEntry("ledMode").forceSetValue(1);
    }

    public static double getX() {
        return table.getEntry("tx").getDouble(0.0);
    }

    /**
     * Angle error in x axis (left or right)
     */
    public double getXError() {
        return tx.getDouble(0.0);
    }

    public double getY() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public static double getA() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public static boolean hasValidTargets() {
        if (table.getEntry("tv").getDouble(0.0) == 1) {
            return true;
        }
        return false;
    }

    public static void changePipeline(int pipeline_num) {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        pipeline.setValue(pipeline_num);
    }

    public static double getContourArea() {
        return table.getEntry("ta0").getDouble(0.0);
    }

    public static double getContourX() {
        return table.getEntry("ty1").getDouble(0.0);
    }

    public static void dumbLineup() {
        testFeed();
        double x = Math.abs(getX());
        double power = x * 0.08;
        if (getX() > 0d) {
            DriveSubsystem.drive(-power, 0);
        }
        if (getX() < -0d) {
            DriveSubsystem.drive(0, power);
        }
    }

    public double findClosestDistance() {
        double myNumber = distanceCalulator(getY());
        double distance = Math.abs(distances[0] - myNumber);
        int idx = 0;
        for (int c = 0; c < distances.length; c++) {
            double cdistance = Math.abs(distances[c] - myNumber);
            if (cdistance < distance) {
                idx = c;
                distance = cdistance;
            }
        }
        return distances[idx];
    }

    public void goToDistance(boolean value) {
        testFeed();
        double distance = distanceCalulator(getY());
        double power = distance * 0.002;
        System.out.println("Im going to " + findClosestDistance());
        {
            if (distance < findClosestDistance() - 3d) { // 5 acts as a range
                DriveSubsystem.drive(power, -power); // should go back if distance is short
            } else if (distance >= findClosestDistance() + 3d) { // 5 acts as a range
                DriveSubsystem.drive(-power, power); // should drive forward
            } else {
                DriveSubsystem.drive(0, 0);
            }
        }
    }

    public static double getPipeline() {
        NetworkTableEntry pipeline = table.getEntry("pipeline");
        return pipeline.getDouble(-1);
    }

    /**
     * 
     * @param ty Ty value of the limelight
     * @return distance from the target
     */
    public static double distanceCalulator(double limelightTY) {

        limelightY = ty.getDouble(0.0);
        double targetAngle = limelightY;
        SmartDashboard.putNumber("Limelight/Target Angle", targetAngle);
        double limelightHeight = RobotContainer.robotConstants.getLimelightConstants().getCameraHeight();
        double targetHeight = RobotContainer.robotConstants.getLimelightConstants().getPowerPortHeight();

        return ((targetHeight - limelightHeight) / (Math.tan(
                (RobotContainer.robotConstants.getLimelightConstants().getCameraAngle() + targetAngle) * Math.PI / 180))
                - RobotContainer.robotConstants.getLimelightConstants().getLimelightOffset());
    }

    public void pushPeriodic() {
        SmartDashboard.putNumber("Limelight Distance", distanceCalulator(getY()));
        SmartDashboard.putNumber("RPM Setpoint", findRPM());
    }

    public double findRPM() {
        double myNumber = distanceCalulator(getY());
        double distance = Math.abs(distances[0] - myNumber);
        int idx = 0;
        for (int c = 0; c < distances.length; c++) {
            double cdistance = Math.abs(distances[c] - myNumber);
            if (cdistance < distance) {
                idx = c;
                distance = cdistance;
            }
        }

        if (idx < distances.length && idx < RPMs.length) {
            return RPMs[idx];
        } else {
            return 0;
        }
    }

}
