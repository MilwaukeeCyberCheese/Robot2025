// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FilteredController;
import frc.robot.utils.FilteredJoystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();

    // The driver's controller
    FilteredController m_operatorController = new FilteredController(OIConstants.kOperatorControllerPort);
    FilteredJoystick m_leftJoystick = new FilteredJoystick(Constants.OIConstants.kLeftJoystickPort);
    FilteredJoystick m_rightJoystick = new FilteredJoystick(Constants.OIConstants.kRightJoystickPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();

        m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem,
                m_rightJoystick::getX,
                m_rightJoystick::getY, m_leftJoystick::getX,
                () -> false,
                Constants.DriveConstants.kRateLimitsEnabled, m_rightJoystick::getButtonTwo,
                m_rightJoystick::getThrottle));

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
        // button 7 on right joystick sets wheels to x
        new Trigger(m_rightJoystick::getButtonSeven).whileTrue(m_driveSubsystem.run(() -> m_driveSubsystem.setX()));

        // zero gyro on right joystick button 5
        new Trigger(m_rightJoystick::getButtonFive)
                .onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.zeroHeading()));

        new Trigger(m_operatorController::getLeftBumper).and(m_operatorController::getRightBumper)
                .onTrue(new Command() {
                    @Override
                    public void initialize() {
                        CommandScheduler.getInstance().cancelAll();
                    }
                });
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