package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import java.io.*;
import java.util.ArrayList;

public class ReadCsvTrajectory {
    public static ArrayList<Pose2d> readCsv(String filename) {
        BufferedReader reader;
        String fileLine;
        String[] fields;
        ArrayList<Pose2d> trajectoryPoints = new ArrayList<>();

        try {
            reader = new BufferedReader(new FileReader(filename));
            while((fileLine = reader.readLine()) != null) {
                fields = fileLine.split(",");
                trajectoryPoints.add(new Pose2d(Units.feetToMeters(Double.parseDouble(fields[0])),
                                                Units.feetToMeters(Double.parseDouble(fields[1])),
                                                Rotation2d.fromDegrees(Double.parseDouble(fields[2]))));

            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return  trajectoryPoints;
    }
}
