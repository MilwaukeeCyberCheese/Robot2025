// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final SparkMaxSim m_drivingSparkSim;
  private final SparkMaxSim m_turningSparkSim;
  private final FlywheelSim m_driveFlywheel;
  private final FlywheelSim m_turningFlywheel;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double driveVelocity = 0;
  private double turnVelocity = 0;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built
   * with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingSparkSim = new SparkMaxSim(m_drivingSpark, DCMotor.getNeoVortex(1));
    m_turningSparkSim = new SparkMaxSim(m_turningSpark, DCMotor.getNeo550(1));
    m_drivingSparkSim.setPosition(0);
    m_turningSparkSim.setPosition(0);

    // TODO: get these values from the robot itself
    // heavier load, higher inertia
    m_driveFlywheel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            DriveConstants.SimConstants.DriveMotor.kV * ModuleConstants.kWheelDiameterMeters / 2,
            DriveConstants.SimConstants.DriveMotor.kA * ModuleConstants.kWheelDiameterMeters / 2),
        DCMotor.getNeoVortex(1));

    // lighter load, lower inertia
    m_turningFlywheel = new FlywheelSim(
        LinearSystemId.identifyVelocitySystem(
            DriveConstants.SimConstants.SteerMotor.kV,
            DriveConstants.SimConstants.SteerMotor.kA),
        DCMotor.getNeo550(1));

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    resetEncoders();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(
        Constants.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(
        Constants.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the drive motor output.
   *
   * @param voltage
   */
  public void setDriveVoltage(double voltage) {
    m_drivingSpark.setVoltage(voltage);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(
        correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void simulationPeriodic(double dt) {
    double driveVoltage = m_drivingSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    double turnVoltage = m_turningSparkSim.getAppliedOutput() * RoboRioSim.getVInVoltage();

    m_driveFlywheel.setInputVoltage(driveVoltage);
    m_turningFlywheel.setInputVoltage(turnVoltage);
    m_driveFlywheel.update(dt);
    m_turningFlywheel.update(dt);

    driveVelocity = m_driveFlywheel.getAngularVelocityRadPerSec();
    turnVelocity = m_turningFlywheel.getAngularVelocityRadPerSec();

    double vbus = RoboRioSim.getVInVoltage();

    m_drivingSparkSim.iterate(driveVelocity, vbus, dt);
    m_turningSparkSim.iterate(turnVelocity, vbus, dt);
  }

  public double getDriveVelocity() {
    return driveVelocity;
  } 

  public double getTurnVelocity() {
    return turnVelocity;
  }
}
