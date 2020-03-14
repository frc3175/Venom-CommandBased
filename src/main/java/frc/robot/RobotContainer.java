/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.net.NetworkInterface;

import frc.robot.auton.SimpleThreeBall;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.commands.FoldCommand;
import frc.robot.commands.FoldSetCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.VisionTrack;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.Constants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final ShooterSubsystem m_ShooterSubsystem;
  public final ClimberSubsystem m_ClimberSubsystem;
  public final IntakeSubsystem m_IntakeSubsystem;
  public final DriveSubsystem m_robotDrive;
  //public final DiagnosticsSubsystem m_DiagnosticsSubsystem;

  // The driver's controller
  public static XBoxController driver;
  public static XBoxController manip;
  public static XBoxController climber;

  // The constants of the robot
  public static Constants robotConstants;

  // Define the possible robot MAC addresses so we can identify what robot we are
  // using.
  private static final byte[] COMPETITION_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x28, (byte) 0x5B,
      (byte) 0x7A };
  private static final byte[] PRACTICE_BOT_MAC_ADDRESS = new byte[] { 0x00, (byte) 0x80, 0x2f, 0x17, (byte) 0xe4,
      (byte) 0x4e };

  public SendableChooser<Command> chooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    boolean competitionBot = false;
    boolean practiceBot = false;

    List<byte[]> macAddresses;
    try {
      macAddresses = getMacAddresses();
    } catch (IOException e) {
      // Don't crash, just log the stacktrace and continue without any mac addresses.
      DriverStation.reportError("Error Retrieving Mac Addresses", false);
      macAddresses = new ArrayList<>();
    }

    for (byte[] macAddress : macAddresses) {
      // First check if we are the competition bot
      if (Arrays.compare(COMPETITION_BOT_MAC_ADDRESS, macAddress) == 0) {
        competitionBot = true;
        break;
      }

      // Next check if we are the practice bot
      if (Arrays.compare(PRACTICE_BOT_MAC_ADDRESS, macAddress) == 0) {
        practiceBot = true;
        break;
      }
    }

    if (!competitionBot && !practiceBot) {
      String[] macAddressStrings = macAddresses.stream().map(RobotContainer::macToString).toArray(String[]::new);

      SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
      SmartDashboard.putString("Competition Bot MAC Address", macToString(COMPETITION_BOT_MAC_ADDRESS));
      SmartDashboard.putString("Practice Bot MAC Address", macToString(PRACTICE_BOT_MAC_ADDRESS));

      // If something goes terribly wrong we still want to use the competition bot
      // stuff in competition.
      competitionBot = true;
    }

    if (competitionBot) {
      robotConstants = new CompConstants();
    }
    if (practiceBot) {
      robotConstants = new PracConstants();
    }

    m_ShooterSubsystem = new ShooterSubsystem();
    m_IntakeSubsystem = new IntakeSubsystem();
    m_ClimberSubsystem = new ClimberSubsystem();
    m_robotDrive = new DriveSubsystem();
    // Configure the button bindings
    configureButtonBindings();
    putAuton();
  }

  public void putAuton() {
    chooser.addOption("AIM AND SHOOT ONLY",
        new SimpleThreeBall(m_robotDrive, m_ShooterSubsystem));
    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driver = new XBoxController(robotConstants.getOIConstants().getKDriverControllerPort());
    manip = new XBoxController(robotConstants.getOIConstants().getKOpControllerPort());
    climber = new XBoxController(robotConstants.getOIConstants().getKClimberControllerPort());

    driver.buttonA.whenHeld(new VisionTrack(m_robotDrive));

    manip.buttonB.whenHeld(new ShooterCommand(m_ShooterSubsystem, 0.7)); // Shoot
    manip.buttonY.whenHeld(new IntakeCommand(m_IntakeSubsystem, m_ShooterSubsystem, false)); // Intake normal
    manip.buttonSELECT.whenHeld(new IntakeCommand(m_IntakeSubsystem, m_ShooterSubsystem, true)); // Intake Reverse

    //TODO: I do not know if this fold command works
    new FoldCommand(m_ClimberSubsystem, climber.getLeftStickY(), climber.getRightStickY());
    climber.buttonA.whenPressed(new FoldSetCommand(m_ClimberSubsystem));
    climber.buttonLeftBumper.whenPressed(new ClimbUpCommand(m_ClimberSubsystem, 0.85));
    climber.buttonRightBumper.whenPressed(new ClimbDownCommand(m_ClimberSubsystem, 0.85));
  }

  /**
   * Gets the MAC addresses of all present network adapters.
   *
   * @return the MAC addresses of all network adapters.
   */
  private static List<byte[]> getMacAddresses() throws IOException {
    List<byte[]> macAddresses = new ArrayList<>();

    Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();

    NetworkInterface networkInterface;
    while (networkInterfaces.hasMoreElements()) {
      networkInterface = networkInterfaces.nextElement();

      byte[] address = networkInterface.getHardwareAddress();
      if (address == null) {
        continue;
      }

      macAddresses.add(address);
    }

    return macAddresses;
  }

  private static String macToString(byte[] address) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < address.length; i++) {
      if (i != 0) {
        builder.append(':');
      }
      builder.append(String.format("%02X", address[i]));
    }
    return builder.toString();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autonMode = null;
    return (Command) chooser.getSelected();
  }
}
