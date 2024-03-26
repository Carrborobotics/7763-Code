package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotorLeft;
    private CANSparkMax armMotorRight;  

    private SparkPIDController m_leftPidController;
    private SparkPIDController m_RightPidController;

    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    public double kP, kI, kD, kIz, kFF, kMax, kMin, kRot;

    public ArmSubsystem() {
        
        // Create motor controller
        armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftCanId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightCanId, MotorType.kBrushless);
        
        // Return the encoder to default state
        armMotorLeft.restoreFactoryDefaults();
        armMotorRight.restoreFactoryDefaults();

        // Create PID controllers
        m_leftPidController = armMotorLeft.getPIDController();
        m_RightPidController = armMotorRight.getPIDController();

        // Set defaults for PID control
        kP = 0.1;
        kI = 1e-4;
        kD = 0;
        kMax = 0.5;
        kMin = -0.5;
        kRot = 0.5;

        m_leftPidController.setP(kP);
        m_leftPidController.setI(kI);
        m_leftPidController.setD(kD);
        m_leftPidController.setOutputRange(kMin, kMax);

        m_RightPidController.setP(kP);
        m_RightPidController.setI(kI);
        m_RightPidController.setD(kD);
        m_RightPidController.setOutputRange(kMin, kMax);

        SmartDashboard.putNumber("arm/p gain", kP);
        SmartDashboard.putNumber("arm/i gain", kI);
        SmartDashboard.putNumber("arm/d gain", kD);
        SmartDashboard.putNumber("arm/max output", kMax);
        SmartDashboard.putNumber("arm/min output", kMin);
        SmartDashboard.putNumber("arm/rotations", kRot);
    }

    public void rotateArmToAmp() {
        m_leftPidController.setReference(kRot, CANSparkMax.ControlType.kPosition);
        m_RightPidController.setReference(kRot, CANSparkMax.ControlType.kPosition);
    }

     public void rotateArmToBot() {
        m_leftPidController.setReference(-kRot, CANSparkMax.ControlType.kPosition);
        m_RightPidController.setReference(-kRot, CANSparkMax.ControlType.kPosition);
    }   

    @Override
    public void periodic() {

        // Tweaks from elastic 
        double p = SmartDashboard.getNumber("arm/p gain", 0);
        double i = SmartDashboard.getNumber("arm/i gain", 0);
        double d = SmartDashboard.getNumber("arm/d gain", 0);
        double max = SmartDashboard.getNumber("arm/max output", 0);
        double min = SmartDashboard.getNumber("arm/min output", 0);
        double rot = SmartDashboard.getNumber("arm/rotations", 0);

        // Listen to the tweaks
        if ((p != kP)) {
            m_leftPidController.setP(p);
            m_RightPidController.setP(p);
            kP = p;
        }

        if ((i != kI)) {
            m_leftPidController.setI(i);
            m_RightPidController.setI(i);
            kI = i;
        } 

        if ((d != kD)) {
            m_leftPidController.setD(d);
            m_RightPidController.setD(d);
            kD = d;
        } 
        
        if((max != kMax) || (min != kMin)) { 
            m_leftPidController.setOutputRange(min, max); 
            m_RightPidController.setOutputRange(min, max);
            kMin = min; 
            kMax = max; 
        }

        if (rot != kRot) { kRot = rot; }

        SmartDashboard.putNumber("arm/left position", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("arm/right position", m_rightEncoder.getPosition());
    }

}
