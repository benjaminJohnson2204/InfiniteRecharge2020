package frc.robot.simulation;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SimConstants {
    public static final double fieldWidth = 15.980;
    public static final double fieldHieght = 8.210;
    public static final double ballDiameter = 0.1778; // In meters

    public static final double robotWidth = 0.686;
    public static final double robotLength = 0.820;
    public static final double intakeLength = 0.3048;
    public static final double shotSpeed = 10; // in meters/second;

    public static final Pose2d redLoadingStation = new Pose2d(0.238258, 2.554548, new Rotation2d());
    public static final Pose2d blueLoadingStation = new Pose2d(15.732665, 5.646024, new Rotation2d());

    public static final Pose2d[] blueTrenchBallPos = {
            new Pose2d(6.154554,7.506032, new Rotation2d()),
            new Pose2d(7.064754,7.506032, new Rotation2d()),
            new Pose2d(7.993157,7.506032, new Rotation2d()),
    };

    public static final Pose2d blueGoalPose = new Pose2d(0, 5.831, new Rotation2d());
}
