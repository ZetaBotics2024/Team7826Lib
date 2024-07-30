package frc.robot.commands.AutonCommands.ChoreoAutonCommands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.ChoreoAutonConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class ChoreoTrajectoryCommandCreater {

    public static Command createChoreoTrajectoryCommand(String choreoTrajectoryFileName, DriveSubsystem driveSubsystem) {
        ChoreoTrajectory trajectory = Choreo.getTrajectory(choreoTrajectoryFileName);
        return Choreo.choreoSwerveCommand(trajectory, driveSubsystem::getRobotPose,
            new PIDController(ChoreoAutonConstants.kPTranslationPIDConstant,
                ChoreoAutonConstants.kITranslationPIDConstant,
                ChoreoAutonConstants.kDTranslationPIDConstant),
            new PIDController(ChoreoAutonConstants.kPTranslationPIDConstant,
                ChoreoAutonConstants.kITranslationPIDConstant,
                ChoreoAutonConstants.kDTranslationPIDConstant),
            new PIDController(ChoreoAutonConstants.kPRotationPIDConstant,
                ChoreoAutonConstants.kIRotationPIDConstant,
                ChoreoAutonConstants.kDRotationPIDConstant),
            driveSubsystem::drive, ()->{return false;},
            driveSubsystem);
    }
}
