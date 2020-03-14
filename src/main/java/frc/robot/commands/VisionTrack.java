/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.utilities.PIDController;
import frc.robot.utilities.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionTrack extends PIDCommand {
    /**
     * Creates a new VisionTrack.
     */

    DriveSubsystem subsystem;

    public VisionTrack(DriveSubsystem subsystem) {
        super(
                // The controller that the command will use
                new PIDController(1.0, 0, 0.5),
                // This should return the measurement
                () -> subsystem.getVisionAngle(),
                // This should return the setpoint (can also be a constant)
                () -> 0,
                // This uses the output
                output -> {
                    // Use the output here
                    System.out.println(-output);
                    subsystem.arcadeDrive(0, output, false);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(subsystem);
        this.subsystem = subsystem;
        getController().setIntegratorRange(-1, 1);
        getController().setIntegratorZone(2);
        getController().setTolerance(.5);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
