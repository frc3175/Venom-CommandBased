package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.ByteDoubleSolenoid;

public class IntakeSubsystem extends SubsystemBase {

    private static TalonFX IntakeFalcon = new TalonFX(
            RobotContainer.robotConstants.getRobotIDConstants().getIntakeMotorID());
    Compressor compressor = new Compressor(RobotContainer.robotConstants.getRobotIDConstants().getPCMID());
    ByteDoubleSolenoid intakeSolenoid = new ByteDoubleSolenoid(
            RobotContainer.robotConstants.getRobotIDConstants().getPCMID(),
            RobotContainer.robotConstants.getRobotIDConstants().getIntakeSolenoidOn(),
            RobotContainer.robotConstants.getRobotIDConstants().getIntakeSolenoidOff(),
            RobotContainer.robotConstants.getIntakeConstants().getDefaultIntakeDirection());

    public IntakeSubsystem() {
        setCompressor(true);

        IntakeFalcon.setInverted(RobotContainer.robotConstants.getIntakeConstants().getIntakeMotorInverted());
    }

    /**
     * 
     * @param power Intake Cells at -1 > 1 power
     */
    public void intakePowerCell(double power) {
        IntakeFalcon.set(ControlMode.PercentOutput, power);
    }

    /**
     * Moves intake up
     */
    public boolean IntakeUp() {
        intakeSolenoid.set(Value.kForward);
        return true;
    }

    /**
     * Moves intake down
     */
    public void IntakeDown() {
        intakeSolenoid.set(Value.kReverse);
    }

    /**
     * 
     * @param onOff false for compressor off true for compressor on.
     */
    public void setCompressor(boolean onOff) {
        if (onOff)
            compressor.start();
        else
            compressor.stop();
    }

    /**
     * 
     * @return intake state
     */
    public boolean isIntakeUp() {
        if (IntakeUp()) {
            return true;
        } else {
            return false;
        }
    }

    // Diagnostic Information used in diagnostic subsystem
    public static boolean isCellIntakeAlive() {
        return (IntakeFalcon.getBusVoltage() != 0.0);
    }

    public static double getTempCellIntake() {
        return IntakeFalcon.getTemperature();
    }

    public static double getIntakeCurrent() {
        return IntakeFalcon.getSupplyCurrent();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}