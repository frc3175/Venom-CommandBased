/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonCommands.TurnToAngle;
import frc.robot.subsystems.DriveSubsystem;

public class Turn90DegreesTest extends SequentialCommandGroup {
    /**
     * Creates a new shoot auton with just the turret
     */
    public Turn90DegreesTest(DriveSubsystem driveSub) {
        // super(new FooCommand(), new BarCommand());
        super(new TurnToAngle(90, driveSub).withTimeout(4));
    }
}