package frc.robot.utils;

public class WaitCommandMilli extends edu.wpi.first.wpilibj2.command.WaitCommand {

    /**
     * Instantiates a new WaitCommand
     * 
     * @param milliseconds
     */
    public WaitCommandMilli(double milliseconds) {
        super(milliseconds / 1000);
    }

}