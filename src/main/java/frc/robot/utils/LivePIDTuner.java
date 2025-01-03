package frc.robot.utils;

import com.revrobotics.SparkPIDController;

/**
 * An object that will automatically update a PID controller's constants based
 * on the SmartDashboard
 */
public class LivePIDTuner {
    private SparkPIDController controller;
    private DashboardUpdater<PIDConstants> pidConstants;

    /**
     * Instantiates a new LivePIDTuner
     * 
     * @param name
     * @param controller
     * @param constants
     */
    public LivePIDTuner(String name, SparkPIDController controller, PIDConstants constants) {
        this.controller = controller;
        pidConstants = new DashboardUpdater<PIDConstants>(name, constants);
    }

    /**
     * Update the pid based off the constants
     */
    public void update() {
        pidConstants.update();
        CustomUtils.setSparkPID(controller, pidConstants.get());
    }

    /**
     * 
     * @return PIDConstants
     */
    public PIDConstants getConstants() {
        pidConstants.update();
        return pidConstants.get();
    }
}
