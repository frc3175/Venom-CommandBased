/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithTurret extends SequentialCommandGroup {
    /**
     * Creates a new Shoot auton with the drivetrain
     */
    public ShootWithTurret(DriveSubsystem driveSub, LimelightSubsystem limelightSub, ShooterSubsystem shooterSub,
            HopperSubsystem hopperSub, double targetServoSpeed) {
        super(new TurretTrack(limelightSub, shooterSub).withTimeout(2),
                new ShooterCommand(shooterSub, hopperSub, limelightSub, targetServoSpeed).withTimeout(4));
    }
}