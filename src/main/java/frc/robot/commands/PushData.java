/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DiagnosticSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class PushData extends CommandBase {
    /**
     * Creates a data pusher.
     */
    DiagnosticSubsystem diagnosticSub;
    ClimberSubsystem climberSub;
    LimelightSubsystem limelightSub;

    public PushData(DiagnosticSubsystem diagnosticSub, ClimberSubsystem climberSub, LimelightSubsystem limelightSub) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(diagnosticSub, climberSub, limelightSub);
        this.diagnosticSub = diagnosticSub;
        this.climberSub = climberSub;
        this.limelightSub = limelightSub;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        diagnosticSub.pushClimberDiagnostics();
        diagnosticSub.pushDriveTrainDiagnostics();
        diagnosticSub.pushIntakeDiagnostics();
        diagnosticSub.pushShooterDiagnostics();
        limelightSub.pushPeriodic();
        climberSub.pushClimberEncoderValue();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
