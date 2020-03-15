/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class VisionTrack extends CommandBase {
    private LimelightSubsystem vision;
    private DriveSubsystem drive;

    private PIDController angleCorrector = new PIDController(0.05, 0, 5);

    /**
     * Creates a new FollowTarget.
     */
    public VisionTrack(LimelightSubsystem limelight, DriveSubsystem rDrive) {
        vision = limelight;
        drive = rDrive;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision, drive);

        angleCorrector.setTolerance(0.2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        angleCorrector.setSetpoint(0);
        vision.forceLEDsOn();
        vision.visionMode();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Passing aim PID output to the drive
        drive.arcadeDrive(0, // Stationary while rotating
                             // Angle Correction
                MathUtil.clamp(
                        // Calculate what to do based off measurement
                        angleCorrector.calculate(-vision.getXError()),
                        // Min, Max output
                        -0.5, 0.5),
                // No squared inputs
                false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        angleCorrector.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}