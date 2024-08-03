package frc.robot.Commands.AutonCommands.ChoreoAutonCommands;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.AutonConstants.ChoreoAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class ChoreoTrajectoryCommandCreater {

    public static Command createChoreoTrajectoryCommand(String choreoTrajectoryFileName, DriveSubsystem driveSubsystem) {
        String allianceColor = "";//RobotModeConstants.isBlueAlliance ? "Blue" : "Red";
        ChoreoTrajectory trajectory = Choreo.getTrajectory(allianceColor + choreoTrajectoryFileName);
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
