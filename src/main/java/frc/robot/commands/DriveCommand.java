package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rotSpeed;
    private final BooleanSupplier m_fieldRelative;
    private final BooleanSupplier m_rateLimit;

    /**
     * Command for driving the robot with dynamic slowing
     * 
     * 
     * @param driveSubsystem subsystem for driving the robot
     *                       {@link frc.robot.subsystems.DriveSubsystem link}
     * @param xSpeed         speed to move on the x-axis
     * @param ySpeed         speed to move on the y-axis
     * @param rotSpeed       rotational speed, positive is counter-clockwise
     * @param fieldRelative  whether commands are relative to the field or the
     *                       robot, true is relative to the field
     * @param rateLimit      whether to enable rate limiting
     * @param slow           whether to slow to 1/4
     * @param throttle       rate to vroom vroom at
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            DoubleSupplier rotSpeed, BooleanSupplier fieldRelative, BooleanSupplier rateLimit, BooleanSupplier slow,
            DoubleSupplier throttle) {

        m_driveSubsystem = driveSubsystem;

        // set slow speeds if necessary and apply throttle limiting
        if (slow.getAsBoolean()) {
            m_xSpeed = () -> xSpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
            m_ySpeed = () -> ySpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
            m_rotSpeed = () -> rotSpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier
                    * Constants.DriveConstants.kTurningSpeedModifier;
        } else {
            m_xSpeed = () -> xSpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
            m_ySpeed = () -> ySpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
            m_rotSpeed = () -> rotSpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0)
                    * Constants.DriveConstants.kTurningSpeedModifier;
        }

        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        // drive robot with provided parameters
        m_driveSubsystem.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), m_rotSpeed.getAsDouble(),
                m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0.0, 0.0, 0.0, m_fieldRelative.getAsBoolean(), m_rateLimit.getAsBoolean());
    }
}