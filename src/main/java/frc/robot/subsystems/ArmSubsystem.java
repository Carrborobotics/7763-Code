package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotorLeft;
    private CANSparkMax armMotorRight;  

    private SparkPIDController m_leftPidController;

    private SparkAbsoluteEncoder m_armEncoder;

    public double kP, kI, kD, kIz, kFF, kMax, kMin, kRot;
    public double  m_startpos;

    public ArmSubsystem() {
        
        // Create motor controller
        armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftCanId, MotorType.kBrushless);
        armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightCanId, MotorType.kBrushless);
        
        // Return the encoder to default state
        armMotorLeft.restoreFactoryDefaults();
        armMotorRight.restoreFactoryDefaults();

        // Have motor follow the other with 1 encoder
        armMotorRight.follow(armMotorLeft, true);

        // Create PID controllers
        m_leftPidController = armMotorLeft.getPIDController();

        // Setup throughbore encoder
        m_armEncoder = armMotorLeft.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_armEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        m_armEncoder.setZeroOffset(Constants.ArmConstants.kZeroOffset);
        m_leftPidController.setFeedbackDevice(m_armEncoder);

        // Setup PID controllers
        m_leftPidController.setPositionPIDWrappingEnabled(true);
        m_leftPidController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        m_leftPidController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // Keep when browning out
        armMotorLeft.burnFlash();
        armMotorRight.burnFlash();

        // Set defaults for PID control
        kP = 1;
        kI = 0;//1e-4;
        kD = 0;
        kMax = 1;
        kMin = -1;
        kRot = 0.3;

        m_leftPidController.setP(kP);
        m_leftPidController.setI(kI);
        m_leftPidController.setD(kD);
        m_leftPidController.setOutputRange(kMin, kMax);

        SmartDashboard.putNumber("arm/p gain", kP);
        SmartDashboard.putNumber("arm/i gain", kI);
        SmartDashboard.putNumber("arm/d gain", kD);
        SmartDashboard.putNumber("arm/max output", kMax);
        SmartDashboard.putNumber("arm/min output", kMin);
        SmartDashboard.putNumber("arm/zero offset", Constants.ArmConstants.kZeroOffset);
        SmartDashboard.putNumber("arm/rotations", kRot);
        
        // Try to start at 0
        m_leftPidController.setReference(0, CANSparkMax.ControlType.kPosition);

        // Record start position
        m_startpos = m_armEncoder.getPosition();
        SmartDashboard.putNumber("arm/start pos", m_startpos);
    }

    public void rotateArmToAmp() {
        m_leftPidController.setReference(Math.PI * Constants.ArmConstants.kArmSmackScaler, CANSparkMax.ControlType.kPosition);
    }

     public void rotateArmToBot() {
        m_leftPidController.setReference(0, CANSparkMax.ControlType.kPosition);
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
        SmartDashboard.putNumber("arm/position", m_armEncoder.getPosition());

        // Listen to the tweaks
        if ((p != kP)) {
            m_leftPidController.setP(p);
            kP = p;
        }

        if ((i != kI)) {
            m_leftPidController.setI(i);
            kI = i;
        } 

        if ((d != kD)) {
            m_leftPidController.setD(d);
            kD = d;
        } 
        
        if((max != kMax) || (min != kMin)) { 
            m_leftPidController.setOutputRange(min, max); 
            kMin = min; 
            kMax = max; 
        }

        if (rot != kRot) { kRot = rot; }
    }

}
