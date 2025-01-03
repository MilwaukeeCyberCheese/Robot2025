package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An object that will put a value to the SmartDashboard, and then update the
 * value within the object when the value on SmartDashboard changes.
 */
public class DashboardUpdater<E> {
    private String key;
    private Object value;

    /**
     * Instantiates a new DashboardUpdaterAny
     * 
     * @param key
     * @param value
     */
    public DashboardUpdater(String key, E value) {
        this.key = key;
        this.value = value;
        SmartDashboard.putString(key, value.toString());
    }

    /**
     * Check the dashboard and update the value if it has changed
     * 
     * @return whether the value has changed
     */
    public boolean update() {
        // get the string back from SmartDashboard
        String val = SmartDashboard.getString(key, "Throw an exception");

        // throw an exception if we get the default value back
        if (val.equals("Throw an exception")) {
            throw new IllegalArgumentException("DashboardUpdater: No value found for: " + key);
        } else if (val.equals(value.toString())) {
            return false;
        }

        // convert the string to the correct type, and set the value
        value = switch (value.getClass().getSimpleName()) {
            case "Double" -> Double.parseDouble(val);
            case "Integer" -> Integer.parseInt(val);
            case "Boolean" -> Boolean.parseBoolean(val);
            case "PIDConstants" -> PIDConstants.parsePIDConstants(val);
            case "String" -> val;
            default -> throw new IllegalArgumentException(
                    "DashboardUpdater: Unsupported type: " + value.getClass().getSimpleName());
        };

        return true;
    }

    /**
     * Set the value of the DashboardUpdater
     * 
     * @return the previous value
     */
    @SuppressWarnings("unchecked")
    public E set(E value) {
        E old = (E) this.value;
        SmartDashboard.putString(key, value.toString());
        this.value = value;
        return old;

    }

    /** 
     * Get the current value
     */
    @SuppressWarnings("unchecked")
    public E get() {
        return (E) value;
    }
}
