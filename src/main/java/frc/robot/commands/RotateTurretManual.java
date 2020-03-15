/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RotateTurretManual extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    ShooterSubsystem subsystem;
    double value;

    public RotateTurretManual(ShooterSubsystem subsystem, double turretRotation) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.value = turretRotation;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.shooterRotation(value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.shooterRotation(0);
    }
}
