package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * An object that will automatically update a PID controller's constants based on the SmartDashboard
 */
public class LivePIDTuner {
  private SparkMaxConfig controller;
  private DashboardUpdater<PIDConstants> pidConstants;

  /**
   * Instantiates a new LivePIDTuner
   *
   * @param name
   * @param controller
   * @param constants
   */
  public LivePIDTuner(String name, SparkMaxConfig controller, PIDConstants constants) {
    this.controller = controller;
    pidConstants = new DashboardUpdater<PIDConstants>(name, constants);
  }

  /** Update the pid based off the constants */
  public void update() {
    pidConstants.update();
    setSparkPID(controller, pidConstants.get());
  }

  /**
   * @return PIDConstants
   */
  public PIDConstants getConstants() {
    pidConstants.update();
    return pidConstants.get();
  }

  /**
   * Set the PID constants of a SparkPIDController
   *
   * @param controller
   * @param constants
   */
  public static void setSparkPID(SparkMaxConfig controller, PIDConstants constants) {
    controller.closedLoop.p(constants.kP).i(constants.kI).d(constants.kD).velocityFF(constants.kFF);
  }
}
