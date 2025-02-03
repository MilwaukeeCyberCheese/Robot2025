// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem m_drive = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
  private final CoralHandlerSubsystem m_coral = new CoralHandlerSubsystem(m_drive.getSimDrive());

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Configure drive input stream
  SwerveInputStream driveInput = SwerveInputStream.of(
      m_drive.getSwerveDrive(),
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    // Set default drive command
    m_drive.setDefaultCommand(m_drive.driveFieldOriented(driveInput));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Zero gyro with A button
    m_driverController.a().onTrue(Commands.runOnce(m_drive::zeroGyro));

    // Lock wheels with left bumper
    m_driverController
        .rightBumper()
        .onTrue(Commands.runOnce(m_coral::intake));
    m_driverController
        .leftBumper()
        .onTrue(Commands.runOnce(m_coral::outtake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
