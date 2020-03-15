package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * <summary> Code meant for the Climber </summary>
 */
public class ClimberSubsystem extends SubsystemBase {
    private static TalonFX leftClimbTalon, rightClimbTalon;
    private static VictorSPX leftFolder, rightFolder;

    // initializes climber motors
    public ClimberSubsystem() {
        leftClimbTalon = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorLeftTalon());
        rightClimbTalon = new TalonFX(RobotContainer.robotConstants.getRobotIDConstants().getClimbMotorRightTalon());
        leftFolder = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getFolderLeftVictor());
        rightFolder = new VictorSPX(RobotContainer.robotConstants.getRobotIDConstants().getFolderRightVictor());

    }

    public void runUpDownToPosition(double position, double climbSpeed) {
        if (position - leftClimbTalon.getSelectedSensorPosition() > 0) {
            leftClimbTalon.set(ControlMode.PercentOutput, climbSpeed);
            rightClimbTalon.set(ControlMode.PercentOutput, -climbSpeed);
        } else {
            leftClimbTalon.set(ControlMode.PercentOutput, 0.0);
            rightClimbTalon.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * 
     * @return Returns true if the climber is in the up position
     */
    public boolean isClimberUp() {
        if(leftClimbTalon.getSelectedSensorPosition() > RobotContainer.robotConstants.getClimbConstants().getUpPosition()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * 
     * @return Returns true if the climber is in the down position
     */
    public boolean isClimberDown() {
        if(leftClimbTalon.getSelectedSensorPosition() > RobotContainer.robotConstants.getClimbConstants().getDownPosition()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * 
     * @param speed Folding speed
     */
    public void foldSet(double speed) {
        leftFolder.set(ControlMode.PercentOutput, speed);
        rightFolder.set(ControlMode.PercentOutput, -speed);
    }

    public void leftFoldSet(double speed) {
        leftFolder.set(ControlMode.PercentOutput, speed);
    }

    public void rightFoldSet(double speed) {
        rightFolder.set(ControlMode.PercentOutput, -speed);
    }

    // Diagnostic Information pushed to diagnositic subsystem
    public static boolean isLeftClimberTalonAlive() {
        return (leftClimbTalon.getBusVoltage() != 0.0);
    }

    public static boolean isRightClimberTalonAlive() {
        return (rightClimbTalon.getBusVoltage() != 0.0);
    }

    public static double getTempLeftTalon() {
        return leftClimbTalon.getTemperature();
    }

    public static double getTempRightTalon() {
        return rightClimbTalon.getTemperature();
    }

    public static void reset(double climbSpeed) {
        leftClimbTalon.set(ControlMode.PercentOutput, -climbSpeed);
        rightClimbTalon.set(ControlMode.PercentOutput, climbSpeed);
    }

    public static void resetEncoders() {
        leftClimbTalon.setSelectedSensorPosition(0);
    }

    public static void pushClimberEncoderValue() {
        SmartDashboard.putNumber("leftClimbTalon", leftClimbTalon.getSelectedSensorPosition());
    }

}