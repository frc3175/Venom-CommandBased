package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class DiagnosticSubsystem extends SubsystemBase{
    private static NetworkTableInstance inst;
    private static NetworkTable diagnosticTable;
    private static SmartDashboard tuningDashboard;
    private static PowerDistributionPanel pdp;


    // creates a diagnostic table
    public DiagnosticSubsystem(){
        inst = NetworkTableInstance.getDefault();
        diagnosticTable = inst.getTable("datatable");
        pdp=new PowerDistributionPanel();
        inst.setUpdateRate(0.01);

    }

    /**
     * Drive train Diagnostics are pushed to SmartDashboard
     */
    public void pushDriveTrainDiagnostics(){
        pushDouble("dtRightFrontTemp", DriveSubsystem.getRightMotorFrontTemp());
        pushDouble("dtRightBackTemp", DriveSubsystem.getRightMotorBackTemp());

        pushDouble("dtLeftFrontTemp", DriveSubsystem.getLeftMotorFrontTemp());
        pushDouble("dtLeftBackTemp", DriveSubsystem.getLeftMotorBackTemp());
        
        pushDouble("dtRightFrontCurrent",DriveSubsystem.getRightMotorFrontCurrent());
        pushDouble("dtRightBackCurrent",DriveSubsystem.getRightMotorBackCurrent());

        pushDouble("dtLeftFrontCurrent",DriveSubsystem.getLeftMotorFrontCurrent());
        pushDouble("dtLeftBackCurrent",DriveSubsystem.getLeftMotorBackCurrent());
        pushDouble("dtVelocity", DriveSubsystem.getAvgVelocity());
    }

    public void pushIntakeDiagnostics(){
        diagnosticTable.getEntry("PowerCellAlive").setBoolean(IntakeSubsystem.isCellIntakeAlive());

        pushDouble("CellIntakeTemp", IntakeSubsystem.getTempCellIntake());
    }

    public void pushShooterDiagnostics() {
        diagnosticTable.getEntry("TopMotorAlive").setBoolean(ShooterSubsystem.isTopShooterAlive());
        diagnosticTable.getEntry("BottomMotorAlive").setBoolean(ShooterSubsystem.isBottomShooterAlive());
        diagnosticTable.getEntry("HopperAlive").setBoolean(HopperSubsystem.isHopperAlive());

        pushDouble("topMotorTemp", ShooterSubsystem.getTempTopTalon());
        pushDouble("bottomMotorTemp", ShooterSubsystem.getTempBottomTalon());
        pushDouble("HopperTemp", HopperSubsystem.getTempHopperTalon());
    }

    public void pushClimberDiagnostics() {
        diagnosticTable.getEntry("LeftClimber").setBoolean(ClimberSubsystem.isLeftClimberTalonAlive());
        diagnosticTable.getEntry("RightClimber").setBoolean(ClimberSubsystem.isRightClimberTalonAlive());

        pushDouble("LeftClimberTemp", ClimberSubsystem.getTempLeftTalon());
        pushDouble("RightClimberTemp", ClimberSubsystem.getTempRightTalon());
    }

    public void pushDouble(String name, double value){
        diagnosticTable.getEntry(name).setDouble(value);

    }

}