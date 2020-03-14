/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonCommands.AutoAlignCommand;
import frc.robot.autonCommands.AutoShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleThreeBall extends SequentialCommandGroup {
    /**
     * Creates a new Shoot.
     */
    public SimpleThreeBall(DriveSubsystem driveSub, ShooterSubsystem shooterSub) {
        // super(new FooCommand(), new BarCommand());
        super(new AutoAlignCommand(driveSub).withTimeout(2).andThen(() -> driveSub.arcadeDrive(0, 0, false)),
                new AutoShooterCommand(shooterSub, driveSub).withTimeout(3));

    }
}
