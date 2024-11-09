
package frc.robot.Subsystems.GameObjectTracking;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

public class GameObjectTracker  {
    private static PhotonCamera exampleCamera = new PhotonCamera("SPCA2688_AV_Camera");
    
    /**
     * Returns the game object position
     * @return double[]: [gameObjectDistanceMeters, headingToAddToFaceGameObject,
     *                    xDistanceToGameObjectMeters, yDistanceToGameObjectMeters]
     */
    public static double[] getTargetDistanceAndHeading() {
        PhotonPipelineResult cameraResult = exampleCamera.getLatestResult();
        
        if(cameraResult.hasTargets()) {
            double gameObjectDistanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.kExampleVisionCameraToRobotCenter.getY(),
                VisionConstants.kGameObjectHeight, 
                VisionConstants.kExampleVisionCameraToRobotCenter.getRotation().getY(), // Pitch
                Units.degreesToRadians(cameraResult.getBestTarget().getPitch()));
            
            /* Can't figure out whare the 0.5842 is from, 
               it may be the distence from the center camera to the center of the robot in the y axis
            */
            double xDistanceToGameObjectMeters = gameObjectDistanceMeters * Math.sin(Units.degreesToRadians(cameraResult.getBestTarget().getYaw()));
            double yDistanceToGameObjectMeters = gameObjectDistanceMeters * Math.cos(Units.degreesToRadians(cameraResult.getBestTarget().getYaw())) + 0.5842; 
            double headingToAddToFaceGameObject = (Math.atan(xDistanceToGameObjectMeters / yDistanceToGameObjectMeters));
            Logger.recordOutput("ObjectTracking/HeadingToAddToFaceTarget", Units.radiansToDegrees(headingToAddToFaceGameObject));
            Logger.recordOutput("ObjectTracking/TargetDistanceMeters", gameObjectDistanceMeters);
            return new double[] {gameObjectDistanceMeters, headingToAddToFaceGameObject, xDistanceToGameObjectMeters, yDistanceToGameObjectMeters};
        }
        return new double[] {0, 0, 0, 0};   
    }

    
    
}