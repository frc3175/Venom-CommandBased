/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utilities.Utils;

public class DriveCommand extends CommandBase {
    /**
     * Creates a new DriveCommand.
     */
    DriveSubsystem subsystem;

    public DriveCommand(DriveSubsystem subsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        this.subsystem = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("DRIVE INIT");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double linearSpeed = Utils.deadband(RobotContainer.driver.getRawAxis(1),
                RobotContainer.robotConstants.getDriveConstants().getDriveTrainDeadband());
        double curveSpeed = Utils.deadband(-RobotContainer.driver.getRawAxis(4),
                RobotContainer.robotConstants.getDriveConstants().getTurnDeadband());

        subsystem.curvatureDrive(linearSpeed, curveSpeed, RobotContainer.driver.getRightBumper());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
