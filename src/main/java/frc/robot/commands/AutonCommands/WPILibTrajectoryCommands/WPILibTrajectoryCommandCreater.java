package frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;
import frc.robot.Utils.LEDUtils.LEDManager;

public class WPILibTrajectoryCommandCreater extends Command{
    private DriveSubsystem driveSubsystem;

    private Trajectory trajectory;
    private Rotation2d desiredEndAngleRotation2d;

    private double startTime = 0;

    private PIDController translationXPIDController;
    private PIDController translationYPIDController;
    private ProfiledPIDController rotationPIDController;

    private HolonomicDriveController wpiLibDriveController;

    private NetworkTablesTunablePIDConstants wpiConstantsPIDLibTranslationPIDValueTuner;
    private NetworkTablesTunablePIDConstants wpiLibRotationPIDValueTuner;

    private TrajectoryConfig trajectoryConfig;

    private boolean hasSetGoal = false;

    public WPILibTrajectoryCommandCreater(AutonPoint[] points, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        configurePIDs();
        configurePIDTuners();
        configureWPILibDriveController();
        configureTrajectoryConfig();
        
        setUpTrajectoryFromPoints(points);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    public WPILibTrajectoryCommandCreater(String autonName, Rotation2d goalEndRotation, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        configurePIDs();
        configurePIDTuners();
        configureWPILibDriveController();
        configureTrajectoryConfig();
        
        setUpTrajectoryFromPath(autonName, goalEndRotation);

        Logger.recordOutput("Auton/WPILibTrajectory/Trajectory", trajectory);
        addRequirements(this.driveSubsystem);
    }

    private void setUpTrajectoryFromPath(String autonName, Rotation2d goalEndRotation) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + autonName + ".wpilib.json"); 
            this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch(IOException exception) {
            this.trajectory = new Trajectory();
            Logger.recordOutput("ValidWPILIBTrajectoryLoaded", false);
        }
        this.desiredEndAngleRotation2d = goalEndRotation;
    }

    private void setUpTrajectoryFromPoints(AutonPoint[] points) {
        ArrayList<Pose2d> mirroredPoints = new ArrayList<>();
        for(int i = 0; i < points.length; i++) {
            Pose2d mirroredPoint = points[i].getAutonPoint();
            Pose2d modifedPoint = new Pose2d(mirroredPoint.getTranslation(), new Rotation2d());
            mirroredPoints.add(modifedPoint);
        }
        this.desiredEndAngleRotation2d = points[points.length-1].getAutonPoint().getRotation();
        
        this.trajectory = TrajectoryGenerator.generateTrajectory(mirroredPoints, trajectoryConfig);
    }

    private void configureTrajectoryConfig() {
        trajectoryConfig = 
            new TrajectoryConfig(WPILibAutonConstants.kMaxTranslationalSpeedInMetersPerSecond,
            WPILibAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond);
        trajectoryConfig.setReversed(false);

    }

    public void configurePIDs() {
        this.translationXPIDController = new PIDController(
            WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant);

        this.translationYPIDController = new PIDController(
            WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant);

        this.rotationPIDController = new ProfiledPIDController(
            WPILibAutonConstants.kPRotationPIDConstant,
            WPILibAutonConstants.kIRotationPIDConstant,
            WPILibAutonConstants.kDRotationPIDConstant,
            WPILibAutonConstants.kRotationPIDControllerConstraints);
    }

    public void configurePIDTuners() {
        this.wpiConstantsPIDLibTranslationPIDValueTuner = new NetworkTablesTunablePIDConstants("WPILib/TranslationPIDValues",
            WPILibAutonConstants.kPTranslationPIDConstant,
            WPILibAutonConstants.kITranslationPIDConstant,
            WPILibAutonConstants.kDTranslationPIDConstant, 0);

        this.wpiLibRotationPIDValueTuner = new NetworkTablesTunablePIDConstants("WPILib/RotationPIDValues",
            WPILibAutonConstants.kPRotationPIDConstant,
            WPILibAutonConstants.kIRotationPIDConstant,
            WPILibAutonConstants.kDRotationPIDConstant, 0);
    }

    public void configureWPILibDriveController() {
        this.wpiLibDriveController = new HolonomicDriveController(
            this.translationXPIDController, this.translationYPIDController,
            this.rotationPIDController);
        this.wpiLibDriveController.setTolerance(WPILibAutonConstants.kPositionTolorence);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        boolean hasAnyPIDValueChanged = false;
        double[] currentTranslationPIDValues = this.wpiConstantsPIDLibTranslationPIDValueTuner.getUpdatedPIDConstants();
        if(this.wpiConstantsPIDLibTranslationPIDValueTuner.hasAnyPIDValueChanged()) {
            this.translationXPIDController = new PIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2]);

            this.translationYPIDController = new PIDController(
                currentTranslationPIDValues[0],
                currentTranslationPIDValues[1],
                currentTranslationPIDValues[2]);

            hasAnyPIDValueChanged = true;
        }
       
        double[] currentRotationPIDValues = this.wpiLibRotationPIDValueTuner.getUpdatedPIDConstants();
        if(this.wpiLibRotationPIDValueTuner.hasAnyPIDValueChanged()) {
            this.rotationPIDController = new ProfiledPIDController(
                currentRotationPIDValues[0],
                currentRotationPIDValues[1],
                currentRotationPIDValues[2],
                WPILibAutonConstants.kRotationPIDControllerConstraints);
            hasAnyPIDValueChanged = true;
        }
        if(hasAnyPIDValueChanged) {
            configureWPILibDriveController();
            hasAnyPIDValueChanged = false;
        }
    } 
    
    @Override 
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        LEDManager.setSolidColor(new int[] {255, 0, 0});
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        this.hasSetGoal = true;
        Trajectory.State goalState = trajectory.sample(Timer.getFPGATimestamp()-startTime);
        ChassisSpeeds desiredChassisSpeeds = this.wpiLibDriveController.calculate(this.driveSubsystem.getRobotPose(), goalState, desiredEndAngleRotation2d);
        this.driveSubsystem.drive(desiredChassisSpeeds);
        Logger.recordOutput("Auton/WPILibTrajectory/TranlsationDesiredVelXMPS", desiredChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput("Auton/WPILibTrajectory/TranlsationDesiredVelYMPS", desiredChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput("Auton/WPILibTrajectory/RotationDesiredRadsPerSecond", desiredChassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TrajectoryTook: " + (Timer.getFPGATimestamp()-startTime));
        LEDManager.setSolidColor(new int[] {0, 0, 255});        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.wpiLibDriveController.atReference() && this.hasSetGoal;
    }
}

    

