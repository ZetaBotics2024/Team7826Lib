package frc.robot.Subsystems.PoseEstimation;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class OdometryIOOdometryUpdaterThread implements OdometryIO{

    private OdometryUpdaterThread odometryUpdaterThread;
    
     /**
      * Creates a OdometryIOSwerveDrivePoseEstimator object
      * @param driveSubsystem DriveSubsysem: The drive subsystem.
      */
    public OdometryIOOdometryUpdaterThread(OdometryUpdaterThread odometryUpdaterThread) {
        this.odometryUpdaterThread = odometryUpdaterThread;
        this.odometryUpdaterThread.start();
    }

    @Override
    public void updateInputs(OdometryIOInputs inputs) {
        inputs.robotPosition = this.odometryUpdaterThread.getRobotPose();
        Logger.recordOutput("RobotPosition", inputs.robotPosition);

    }  
    
    @Override
    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }
    
    @Override
    public void setRobotPose(Pose2d newRobotPose) {
        this.odometryUpdaterThread.setRobotPose(newRobotPose);
    }

    @Override
    public void resetRobotPose() {
        this.odometryUpdaterThread.resetRobotPose();
    }

}