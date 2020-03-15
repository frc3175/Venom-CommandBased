/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    ShooterSubsystem subsystem;
    LimelightSubsystem limelightSub;
    double targetRPM;
    double targetServoSpeed;

    /**
     * 
     * @param subsystem Shooter Subsystem to use
     * @param limelightSub Limelight subsystem to use
     */
    public AutoShooterCommand(ShooterSubsystem subsystem, LimelightSubsystem limelightSub) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, limelightSub);
        this.subsystem = subsystem;
        this.limelightSub = limelightSub;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.setHoodAngle(20);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double targetVelocity = limelightSub.findRPM();
        subsystem.shoot(targetVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.shoot(0);
        subsystem.setServoSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
