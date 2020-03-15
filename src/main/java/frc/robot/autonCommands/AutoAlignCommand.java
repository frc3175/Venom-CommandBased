/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoAlignCommand extends PIDCommand {
    /**
     * Creates a new VisionTrack.
     */

    DriveSubsystem subsystem;
    LimelightSubsystem limelightSub;

    /**
     * @param subsystem Drive subsystem to use
     * @param limelightSub Limelight subsystem to use
     */
    public AutoAlignCommand(DriveSubsystem subsystem, LimelightSubsystem limelightSub)
  {
    super(
        // The controller that the command will use
        new PIDController(1.0, 0.0, .05),
        // This should return the measurement
        () -> limelightSub.getY(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output ->
        {
          // Use the output here
          //System.out.println(output);
          subsystem.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(subsystem);
    this.subsystem = subsystem;
    getController().setIntegratorRange(-1, 1);
    getController().setTolerance(.02);
  }
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("AutoAlignCommand Ended");
    getController().reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return getController().atSetpoint(); 
  }
}
