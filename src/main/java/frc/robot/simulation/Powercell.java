package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class Powercell {
    double ballDiameter = 0.1778; // In meters
    boolean wasShot;
    Pose2d ballPose = new Pose2d();
    Pose2d redLoadingStation = new Pose2d(0.238258, 2.554548, new Rotation2d());
    Pose2d blueLoadingStation = new Pose2d(15.732665, 5.646024, new Rotation2d());
    double shotSpeed = 10; // in meters/second;
    double m_lastTimestamp;
    int ballState = 1;

    double fieldWidth = 15.980;
    double fieldHieght = 8.210;

    Turret m_turret;
    Field2d m_field2dBall;

    public Powercell(Turret turret) {
        m_turret = turret;
        m_field2dBall = new Field2d();
    }

    public int getBallState() {
        return ballState;
    }

    public boolean getBallShotState() {
        return wasShot;
    }

    public void setBallShotState(boolean shotState) {
        wasShot = shotState;
    }

    public void updateBallState() {
//        System.out.println("Ball Shot: " + wasShot + "\tBall State: " + ballState);
//        System.out.println("Ball State: " + ballState + "\tCos: " + ballPose.getRotation().getCos() + "\tX Pos: " + ballPose.getX());
        switch (ballState) {
            case 3:
                // Ball is out of bounds
                if(ballPose.getX() < fieldWidth / 2.0)
                    ballPose = redLoadingStation;
                else
                    ballPose = blueLoadingStation;

                ballState = 0;
                break;
            case 2:
                // Ball is traveling in the air
                double currentTime = RobotController.getFPGATime();
                // FPGA time is in microseonds, need to convert it into seconds
                double deltaT = (currentTime - m_lastTimestamp) / 1e6;
                double distanceTraveled = shotSpeed * deltaT;
                double deltaX = distanceTraveled * ballPose.getRotation().getCos();
                double deltaY = distanceTraveled * ballPose.getRotation().getSin();
//                System.out.println("Delta X: " + deltaX + "\tDelta Y: " + deltaY + "\tDelta T: " + deltaT);
                ballPose = new Pose2d(deltaX + ballPose.getX(),
                                      deltaY + ballPose.getY(),
                                          ballPose.getRotation());

                m_lastTimestamp = currentTime;

                if(ballPose.getX() < 0 || ballPose.getX() > fieldWidth ||
                   ballPose.getY() < 0 || ballPose.getX() > fieldHieght)
                    ballState = 3;
                break;
            case 1:
                // Ball has been picked up by the robot
                ballPose = m_turret.getTurretSimPose();

                // Ball has been shot;
                if(wasShot) {
                    setBallShotState(false);
                    m_lastTimestamp = RobotController.getFPGATime();
                    ballState = 2;
                }
                break;
            case 0:
            default:
                // Ball is on the ground

                ballState = 1;
                break;
        }
    }

    public void simulationPeriodic() {
        m_field2dBall.setRobotPose(ballPose);
        SmartDashboard.putData("Ball", m_field2dBall);
        updateBallState();
    }
}
