/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class FoldCommand extends CommandBase {
    /**
     * Creates a new ClimbCommand.
     */
    ClimberSubsystem subsystem;
    double percentOutputL = 0;
    double percentOutputR = 0;

    public FoldCommand(ClimberSubsystem subsystem, double percentOutputL, double percentOutputR) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.percentOutputL = percentOutputL;
        this.percentOutputR = percentOutputR;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.leftFoldSet(percentOutputL);
        subsystem.rightFoldSet(percentOutputR);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.leftFoldSet(0);
        subsystem.rightFoldSet(0);
    }

}
