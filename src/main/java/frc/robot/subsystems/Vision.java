package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{
    private final PhotonCamera m_camera;

    public Vision() {
        m_camera = new PhotonCamera("photonvision");

    }

    public boolean hasTarget(){
        return m_camera.getLatestResult().hasTargets();
        //return (m_camera.getLatestResult() != null) ? m_camera.getLatestResult().hasTargets() : false;
    }

    public PhotonTrackedTarget getTarget(){
        return (hasTarget()) ? (m_camera.getLatestResult().getBestTarget()) : null;
    }

    public int targetID(){
        return (hasTarget()) ? (getTarget().getFiducialId()) : 0;
    }

    public double getYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

    public double getArea(PhotonTrackedTarget target){
        return target.getArea();
    }

    public boolean targetAreaReached() {
        return (getArea(getTarget()) < VisionConstants.kCameraTargetArea) ? true : false; // need to figure out the area to slow down at
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
        if (result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kCamHeight, 
                VisionConstants.kTagHeight,
                VisionConstants.kCamPitch,
                Units.degreesToRadians(target.getPitch())
            );
            Shuffleboard.getTab("Vision").add("April tag Targetted", result.hasTargets());
            Shuffleboard.getTab("Vision").add("April Yaw", getTarget().getYaw());
            Shuffleboard.getTab("Vision").add("April Area", getTarget().getArea());
            Shuffleboard.getTab("Vision").add("April Skew", getTarget().getSkew());
            Shuffleboard.getTab("Vision").add("April Range", range);
            Shuffleboard.getTab("Vision").add("April ID", Double.valueOf(targetID()));
            Shuffleboard.getTab("Vision").add("Amp Target", goodTarget(targetID()));
        }
    }
}
