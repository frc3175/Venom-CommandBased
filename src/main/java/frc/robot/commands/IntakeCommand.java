/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    /**
     * Creates a new IntakeCommand.
     */
    IntakeSubsystem subsystem;
    HopperSubsystem hopperSub;
    /**
     * Boolean values
     */
    boolean shouldRunBack;
    
    Timer agitatorTimer = new Timer(); //Agitation Timer

    public IntakeCommand(IntakeSubsystem subsystem, HopperSubsystem hopperSub, boolean shouldRunBack) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, hopperSub);
        this.subsystem = subsystem;
        this.hopperSub = hopperSub;
        this.shouldRunBack = shouldRunBack;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.IntakeDown();
        agitatorTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!subsystem.isIntakeUp()) {
            if (this.shouldRunBack) {
                subsystem.intakePowerCell(RobotContainer.robotConstants.getIntakeConstants().getIntakePercentOutput());
            } else {
                subsystem.intakePowerCell(-RobotContainer.robotConstants.getIntakeConstants().getIntakePercentOutput());
            }
        } else {
            if (agitatorTimer.get() < 1) { // For 1 second...
                hopperSub.hopperPower(RobotContainer.robotConstants.getIntakeConstants().getHopperSpeedReverse());
            } else if (agitatorTimer.get() < 3) { // For 3 seconds...
                hopperSub.hopperPower(RobotContainer.robotConstants.getIntakeConstants().getHopperSpeedForward());
            } else {
                agitatorTimer.reset(); // Reset timer to 0 seconds
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.IntakeUp();
        subsystem.intakePowerCell(0);
        hopperSub.hopperPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
