package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    private final PhotonCamera m_camera;

    public Vision() {
        m_camera = new PhotonCamera("photonvision");
        m_camera.setPipelineIndex(0);
    }

    public boolean hasTarget(){
        return m_camera.getLatestResult().hasTargets();
    }

    public boolean noTarget(){
        return !(m_camera.getLatestResult().hasTargets());
    }

    public PhotonTrackedTarget getTarget(){
        return (hasTarget()) ? (m_camera.getLatestResult().getBestTarget()) : null;
    }

    public int targetID(PhotonTrackedTarget target){
        return target.getFiducialId();
    }

    public double getYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

    public double getArea(PhotonTrackedTarget target){
        return target.getArea();
    }

    public boolean targetAreaReached() {
        if (m_camera.getLatestResult().getBestTarget() != null){
            return (m_camera.getLatestResult().getBestTarget().getArea() > VisionConstants.kCameraTargetArea) ? true : false; // need to figure out the area to slow down at
        }
        return false;
    }

    public double getSkew(PhotonTrackedTarget target){
        return target.getSkew();
    }

    public void setLED(VisionLEDMode mode){
        m_camera.setLED(mode);
    }

    public boolean goodTarget(int id){
        // Amp is 5 or 6 depending on red/blue, either is fine for now
        return (id == 5 || id == 6) ? true : false;
    }

    @Override
    public void periodic(){
        var result = m_camera.getLatestResult();
        SmartDashboard.putBoolean("April Tag", result.hasTargets());
        if (result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kCamHeight, 
                VisionConstants.kTagHeight,
                VisionConstants.kCamPitch,
                Units.degreesToRadians(target.getPitch())
            );
            SmartDashboard.putNumber("April Yaw", result.getBestTarget().getYaw());
            SmartDashboard.putNumber("April Area", result.getBestTarget().getArea());
            SmartDashboard.putNumber("April Skew", result.getBestTarget().getSkew());
            SmartDashboard.putNumber("April Range", range);
            SmartDashboard.putNumber("April ID", Double.valueOf(targetID(target)));
        }
    }
}
