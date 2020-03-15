package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HopperSubsystem extends SubsystemBase {

    private static TalonSRX hopperTalon;

    public HopperSubsystem() {

        hopperTalon = new TalonSRX(RobotContainer.robotConstants.getRobotIDConstants().getHopperMotorID());
    }


    public void hopperPower(double power) {
        hopperTalon.set(ControlMode.PercentOutput, power);
    }


    //Diagnostics
    public static double getTempHopperTalon() {
        return hopperTalon.getTemperature();
    }


    public static boolean isHopperAlive() {
        return (hopperTalon.getBusVoltage() != 0.0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
}