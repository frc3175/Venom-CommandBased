/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbUpCommand extends CommandBase {
    /**
     * Creates a new ClimbCommand.
     */
    ClimberSubsystem subsystem;
    double percentOutput = 0;

    public ClimbUpCommand(ClimberSubsystem subsystem, double percentOutput) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.percentOutput = percentOutput;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.runUpDownToPosition(RobotContainer.robotConstants.getClimbConstants().getUpPosition(), percentOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return subsystem.isClimberUp();
    }
}
