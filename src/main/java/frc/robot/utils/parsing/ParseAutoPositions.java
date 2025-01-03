package frc.robot.utils.parsing;

import java.nio.file.*;
import org.json.JSONObject;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Stream;

public class ParseAutoPositions {

    public static void main(String args[]) {
        List<String> notFound = new LinkedList<>();
        // YMMV, change the path
        Path start = Paths.get("/home/sam/gitClones/aurelia2024/src/main/deploy/pathplanner/autos");
        // this is the first line of the hashmap path
        System.out.print(
                "public static final HashMap<String, Pose2d> kStartingPositions = new HashMap<String, Pose2d>() {{");
        try (Stream<Path> stream = Files.walk(start, 1)) {
            stream
                    .filter(file -> !Files.isDirectory(file))
                    .forEach(file -> {
                        try {
                            String content = new String(Files.readAllBytes(file));
                            double[] pose = parseJson(content);
                            String filename = file.getFileName().toString();
                            System.out.println("put(\"" + filename.substring(0, filename.length() - 5)
                                    + "\", new Pose2d(" + pose[0] + ", "
                                    + pose[1] + ", new Rotation2d(Math.toRadians(" + pose[2] + "))));");
                        } catch (Exception e) {
                            notFound.add(file.getFileName().toString());
                        }
                    });
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("}};");

        for (String filename : notFound) {
            System.out.println("We didn't find a starting pose for " + filename + ", you should fix that");
        }
    }

    public static double[] parseJson(String content) {
        JSONObject jsonObject = new JSONObject(content);
        JSONObject startingPose = jsonObject.getJSONObject("startingPose");
        JSONObject position = startingPose.getJSONObject("position");
        double x = position.getDouble("x");
        double y = position.getDouble("y");
        double rotation = startingPose.getDouble("rotation");
        return new double[] { x, y, rotation };
    }

}
