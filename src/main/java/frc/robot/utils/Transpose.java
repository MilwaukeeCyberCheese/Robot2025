package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Transpose {
    private static final double fieldLength = 16.54;
    // private static final double fieldWidth = 8.21;

    public static Pose2d transposeToRed(Pose2d input) {
        double newX = (input.getX() > fieldLength / 2.0) ? input.getX() + (fieldLength / 2.0) : input.getX() - (fieldLength / 2.0);
        double newY = input.getY();
        Rotation2d newRotation = input.getRotation().plus(new Rotation2d(Math.PI));
        
        
        return new Pose2d(newX, newY, newRotation);
    }

    public static Pose2d transposeToBlue(Pose2d input) {
        double newX = (input.getX() < fieldLength / 2.0) ? input.getX() + (fieldLength / 2.0) : input.getX() - (fieldLength / 2.0);
        double newY = input.getY();
        Rotation2d newRotation = input.getRotation().plus(new Rotation2d(Math.PI));
        
        
        return new Pose2d(newX, newY, newRotation);
    }
}
