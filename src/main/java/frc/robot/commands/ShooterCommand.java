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
import frc.robot.utilities.Limelight;

public class ShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    ShooterSubsystem subsystem;
    double targetServoSpeed;

    public ShooterCommand(ShooterSubsystem subsystem, double targetServoSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.subsystem = subsystem;
        this.targetServoSpeed = targetServoSpeed;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        subsystem.shoot(true);
        SmartDashboard.putNumber("Shooter Target Velocity", Limelight.findRPM());
        subsystem.setHoodAngle(30);
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
