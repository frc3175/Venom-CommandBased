package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
/**
 * <summary> Code meant for the Shooter and launcher balls with the hopper
 * </summary>
 */
public class ShooterSubsystem extends SubsystemBase {
    private static TalonSRX masterShooterTalon, followerTalon, hopperTalon, rotationTalon;
    private static Servo hoodServo;

    double hoodAngle = 0;

    PIDController pidController = new PIDController(0.1, 0, 0);

    // initializes the motors
    public ShooterSubsystem() {
        masterShooterTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getTopShooterMotorID());
        followerTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getBottomShooterMotorID());
        hopperTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getHopperMotorID());
        rotationTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getRotationMotorID());
        hoodServo = new Servo(RobotContainer.robotConstants.getRobotIDConstants().getHoodedAngleMotorID());

        masterShooterTalon.configFactoryDefault();
        masterShooterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.setSensorPhase(true);
        masterShooterTalon.configNominalOutputForward(0,
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.configNominalOutputReverse(0,
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.configPeakOutputForward(1, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.configPeakOutputReverse(1, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());

        // Gains
        masterShooterTalon.config_kF(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkF(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.config_kP(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkP(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.config_kI(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkI(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        masterShooterTalon.config_kD(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkD(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());

        followerTalon.configFactoryDefault();
        followerTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.setSensorPhase(true);
        followerTalon.configNominalOutputForward(0, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.configNominalOutputReverse(0, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.configPeakOutputForward(1, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.configPeakOutputReverse(1, RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());

        // Gains
        followerTalon.config_kF(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkF(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.config_kP(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkP(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.config_kI(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkI(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());
        followerTalon.config_kD(RobotContainer.robotConstants.getPIDConstants().getkPIDLoopIdx(),
                RobotContainer.robotConstants.getPIDConstants().getkD(),
                RobotContainer.robotConstants.getPIDConstants().getkTimeoutMs());

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 45, 1.0);
        masterShooterTalon.configSupplyCurrentLimit(currentConfig);
        followerTalon.configSupplyCurrentLimit(currentConfig);

    }

    public void shoot(boolean pressed) {
        if (pressed) {
            double targetVelocity_UnitsPer100ms = LimelightSubsystem.findRPM();
            masterShooterTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            followerTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
        } else {
            masterShooterTalon.set(ControlMode.Velocity, 0);
            followerTalon.set(ControlMode.Velocity, 0);
        }
    }

    public static void shooterRotation(double speed) {
        rotationTalon.set(ControlMode.PercentOutput, speed);
    }

    /**
     * 
     * @param angle the angle you want the hood to be at. (Zero will shoot the ball
     *              high and 60 will shoot it low almost into the floor.) a value of
     *              less than 0 or greater than 60 will disable pid.
     */
    public void setHoodAngle(double angle) {
        this.hoodAngle = angle;
        pidController.setSetpoint(angle);
    }

    public void setServoSpeed(double speed) {
        speed = (speed / 2.0) + 0.5;
        hoodServo.set(speed);
    }

    //TODO: Calculate a full rotation and change it to degrees
    public void setTurretAngle(double setpoint) {
        LimelightSubsystem.limelightPID.setSetpoint(setpoint);
    }

    public double getTurretEncoder() {
        return rotationTalon.getSelectedSensorPosition();
    }

    public static boolean reachedRPM() {
        for (int i = 0; i < LimelightSubsystem.RPMs.length; i++) {
            if (followerTalon.getSelectedSensorVelocity() >= LimelightSubsystem.RPMs[i] - 100
                    && followerTalon.getSelectedSensorVelocity() <= LimelightSubsystem.RPMs[i] + 100) {
                return true;
            }
        }
        return false;
    }

    public void hopperPower(double power) {
        hopperTalon.set(ControlMode.PercentOutput, power);
    }

    public static double publishRPM() {
        SmartDashboard.putNumber("Shooter/TopMotor RPM", masterShooterTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter/BottomMotor RPM", followerTalon.getSelectedSensorVelocity());
        return masterShooterTalon.getSelectedSensorVelocity();
    }

    /**
     * 
     * @return returns the current Hood Angle in Degrees.
     */
    public double getHoodAngle() {
        return hoodServo.getAngle();
    }

    // Diagnostic Information used in diagnostics subsystem
    public static boolean isTopShooterAlive() {
        return (masterShooterTalon.getBusVoltage() != 0.0);
    }

    public static boolean isBottomShooterAlive() {
        return (followerTalon.getBusVoltage() != 0.0);
    }

    public static boolean isHopperAlive() {
        return (hopperTalon.getBusVoltage() != 0.0);
    }

    public static double getTempTopTalon() {
        return masterShooterTalon.getTemperature();
    }

    public static double getTempBottomTalon() {
        return followerTalon.getTemperature();
    }

    public static double getTempHopperTalon() {
        return hopperTalon.getTemperature();
    }

    @Override
    public void periodic() {

        setServoSpeed(pidController.calculate(getHoodAngle()));
        SmartDashboard.putNumber("Curr Angle", getHoodAngle());
        setTurretAngle(LimelightSubsystem.limelightPID.calculate(getTurretEncoder()));
    }
}