/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    ShooterSubsystem subsystem;
    double targetRPM;
    DriveSubsystem driveSub;
    double targetServoSpeed;

    public AutoShooterCommand(ShooterSubsystem subsystem, DriveSubsystem driveSub) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, driveSub);
        this.subsystem = subsystem;
        this.driveSub = driveSub;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.setHoodAngle(20);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        subsystem.shoot(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.shoot(false);
        subsystem.setServoSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
