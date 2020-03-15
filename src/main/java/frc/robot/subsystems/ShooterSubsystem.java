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
import frc.robot.utilities.Gains;

/**
 * <summary> Code meant for the Shooter and launcher balls with the hopper
 * </summary>
 */
public class ShooterSubsystem extends SubsystemBase {
    private static TalonSRX masterShooterTalon, followerTalon, rotationTalon;
    private static Servo hoodServo;

    double hoodAngle = 0;

    PIDController pidController = new PIDController(0.1, 0, 0);

    // initializes the motors
    public ShooterSubsystem() {
        masterShooterTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getTopShooterMotorID());
        followerTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getBottomShooterMotorID());
        rotationTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getRotationMotorID());
        hoodServo = new Servo(RobotContainer.robotConstants.getRobotIDConstants().getHoodedAngleMotorID()); //TODO: plug in servo to PWM port 0

        masterShooterTalon.configFactoryDefault();
        masterShooterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.setSensorPhase(true);
        masterShooterTalon.configNominalOutputForward(0, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.configNominalOutputReverse(0, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.configPeakOutputForward(1, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.configPeakOutputReverse(1, Gains.shooterPID.kTimeoutMs);

        // Gains
        masterShooterTalon.config_kF(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kF, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.config_kP(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kP, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.config_kI(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kI, Gains.shooterPID.kTimeoutMs);
        masterShooterTalon.config_kD(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kD, Gains.shooterPID.kTimeoutMs);

        followerTalon.configFactoryDefault();
        followerTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kTimeoutMs);
        followerTalon.setSensorPhase(true);
        followerTalon.configNominalOutputForward(0, Gains.shooterPID.kTimeoutMs);
        followerTalon.configNominalOutputReverse(0, Gains.shooterPID.kTimeoutMs);
        followerTalon.configPeakOutputForward(1, Gains.shooterPID.kTimeoutMs);
        followerTalon.configPeakOutputReverse(1, Gains.shooterPID.kTimeoutMs);

        // Gains
        followerTalon.config_kF(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kF, Gains.shooterPID.kTimeoutMs);
        followerTalon.config_kP(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kP, Gains.shooterPID.kTimeoutMs);
        followerTalon.config_kI(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kI, Gains.shooterPID.kTimeoutMs);
        followerTalon.config_kD(Gains.shooterPID.kPIDLoopIdx, Gains.shooterPID.kD, Gains.shooterPID.kTimeoutMs);

        SupplyCurrentLimitConfiguration currentConfig = new SupplyCurrentLimitConfiguration(true, 40, 45, 1.0);
        masterShooterTalon.configSupplyCurrentLimit(currentConfig);
        followerTalon.configSupplyCurrentLimit(currentConfig);

    }

    public void shoot(double targetVelocity) {
        masterShooterTalon.set(ControlMode.Velocity, targetVelocity);
        followerTalon.set(ControlMode.Velocity, targetVelocity);
        SmartDashboard.putNumber("Shooter/TopMotor RPM", masterShooterTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter/BottomMotor RPM", followerTalon.getSelectedSensorVelocity());
    }

    public void shooterRotation(double speed) {
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

    public static double getTempTopTalon() {
        return masterShooterTalon.getTemperature();
    }

    public static double getTempBottomTalon() {
        return followerTalon.getTemperature();
    }

    @Override
    public void periodic() {

        setServoSpeed(pidController.calculate(getHoodAngle()));
        SmartDashboard.putNumber("Curr Angle", getHoodAngle());
    }
}