/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class ShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    ShooterSubsystem subsystem;
    HopperSubsystem hopperSubsystem;
    LimelightSubsystem limelightSubsystem;
    double targetServoSpeed;

    public ShooterCommand(ShooterSubsystem subsystem, HopperSubsystem hopperSubsystem, LimelightSubsystem limelightSubsystem, double targetServoSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem, hopperSubsystem, limelightSubsystem);
        this.subsystem = subsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.targetServoSpeed = targetServoSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        double targetVelocity = limelightSubsystem.findRPM();
        subsystem.shoot(targetVelocity);
        hopperSubsystem.hopperPower(RobotContainer.robotConstants.getIntakeConstants().getHopperSpeedForward());
        SmartDashboard.putNumber("Shooter Target Velocity", limelightSubsystem.findRPM());
        subsystem.setHoodAngle(30);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.shoot(0);
        subsystem.setServoSpeed(0);
        hopperSubsystem.hopperPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
