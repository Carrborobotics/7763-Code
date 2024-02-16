package frc.robot.subsystems;

import java.security.Key;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    
    // Setup basics objects for the limelight
    private final NetworkTable m_lime;
    private final String m_name;
    private final ArrayList<double[]> m_poses = new ArrayList();

    public Limelight(String name) {
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledmode").setDouble(3.0);
        m_name = name;
    }

    // Override some methods for using the limelight
    @Override
    public void periodic() {

    }

    // poses
    public void storePose(double[] pose) {
        if (pose != new double[7]) {
            m_poses.add(pose);
        }
    }

    // Deal with the pipeline indexes
    public void setPipeline(int pipelineIndex) {
        m_lime.getEntry("pipeline").setDouble((double) pipelineIndex);
    }

    public int getPipeline() {
        return (int) m_lime.getEntry("getpipe").getDouble(-1.0);
    }

    // Return ll name
    public String getName() {
        return m_name;
    }

    // Update LED setting
    public void setLights(int status) {
        m_lime.getEntry("ledmode").setNumber(status);
    }

        //LED on
    public Command ledOn() {
        m_lime.getEntry("ledmode").setNumber(3.0);
        return null;
    }

    //LED off
    public Command ledOff() {
        m_lime.getEntry("ledMode").setNumber(1.0);
        return null;
    }


    //take snapshot
    public Command takeSnap() {
        m_lime.getEntry("snapshot").setNumber(1.0);
        return null;
    }


    

}
