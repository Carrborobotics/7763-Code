// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shooterLeft;
    private CANSparkFlex shooterRight;

    private SparkPIDController m_leftPidController;
    private SparkPIDController m_rightPidController;
    
    private RelativeEncoder m_leftEncoder;
    private RelativeEncoder m_rightEncoder;

    private CANSparkMax intake1;
    private CANSparkMax intake2;
    
    private DigitalInput noteSensor;

    public double shooterSpeedReq;

    public ShooterSubsystem() {

        shooterLeft = new CANSparkMax(Constants.ShooterConstants.kShooterLeftId, MotorType.kBrushless);
        shooterRight = new CANSparkFlex(Constants.ShooterConstants.kShooterRightId, MotorType.kBrushless);

        shooterLeft.restoreFactoryDefaults();
        shooterRight.restoreFactoryDefaults();

        shooterLeft.enableVoltageCompensation(12);
        shooterRight.enableVoltageCompensation(12);
        
        m_leftPidController = shooterLeft.getPIDController();

        m_leftPidController.setP(Constants.ShooterConstants.kPshooter);
        m_leftPidController.setI(Constants.ShooterConstants.kIshooter);
        m_leftPidController.setD(Constants.ShooterConstants.kDshooter);
        m_leftPidController.setFF(Constants.ShooterConstants.kFFshooter);
        m_leftPidController.setOutputRange(Constants.ShooterConstants.kMinShooter, Constants.ShooterConstants.kMaxShooter);

        m_rightPidController = shooterRight.getPIDController();
        
        m_rightPidController.setP(Constants.ShooterConstants.kPshooter);
        m_rightPidController.setI(Constants.ShooterConstants.kIshooter);
        m_rightPidController.setD(Constants.ShooterConstants.kDshooter);
        m_rightPidController.setFF(Constants.ShooterConstants.kFFshooter);
        m_rightPidController.setOutputRange(Constants.ShooterConstants.kMinShooter, Constants.ShooterConstants.kMaxShooter);
        
        m_leftEncoder = shooterLeft.getEncoder();
        m_rightEncoder = shooterRight.getEncoder();

        m_leftPidController.setFeedbackDevice(m_leftEncoder);
        m_rightPidController.setFeedbackDevice(m_rightEncoder);

        intake1 = new CANSparkMax(Constants.ShooterConstants.kintake1Id, MotorType.kBrushless);
        intake2 = new CANSparkMax(Constants.ShooterConstants.kintake2Id, MotorType.kBrushless);

        intake1.restoreFactoryDefaults();
        intake2.restoreFactoryDefaults();

        noteSensor = new DigitalInput(Constants.ShooterConstants.kNoteSensorId);

        shooterSpeedReq = 0;

        // Start up with the intake ON
        //intakeON(1);
    }
    // NOTE: vortex has inverted speed to neo

    public void shooterON(double inputSpeed){
        shooterSpeedReq = inputSpeed * Constants.NeoMotorConstants.kFreeSpeedRpm;
        m_leftPidController.setReference(inputSpeed * Constants.NeoMotorConstants.kFreeSpeedRpm, CANSparkMax.ControlType.kVelocity);
        m_rightPidController.setReference(inputSpeed * Constants.VortexMotorConstants.kFreeSpeedRpm, CANSparkFlex.ControlType.kVelocity);    
    }

    public boolean isShooterReady() {
        return (Math.abs(m_leftEncoder.getVelocity() - shooterSpeedReq) < 50 && 
            (Math.abs(m_rightEncoder.getVelocity() - shooterSpeedReq) < 50)); 
    }

    public void shooterREV(){
        shooterSpeedReq = -1 * Constants.VortexMotorConstants.kFreeSpeedRpm;
        shooterLeft.set(-1);
        shooterRight.set(-1);
    }

    public void shooterOFF() {
        shooterSpeedReq = 0;
        shooterLeft.set(0);
        shooterRight.set(0);
    }

    public void intakeON(double inputSpeed){
        intake1.set(inputSpeed);
        intake2.set(-inputSpeed);
    }
  
    public void intakeOFF(){
        intake1.set(0);
        intake2.set(0);
    }
    
    public void intakeREV(){
        intake1.set(-0.5);
        intake2.set(0.5);        
    }

    // Default to false with note
    public boolean getNoteSensor(){
        return noteSensor.get();
    }

    // Default to true with note
    public boolean getInvNoteSensor(){
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        // Handle checking the note sensor to see if the intake is loaded
        // noteSensor.get() is True if there is not a note
        SmartDashboard.putBoolean("shooter/Note Sensor", noteSensor.get());
        SmartDashboard.putNumber("shooter/Left Voltage", shooterLeft.getBusVoltage());
        SmartDashboard.putNumber("shooter/Right Voltage", shooterRight.getBusVoltage());
        SmartDashboard.putNumber("shooter/Left Velocity", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("shooter/Right Velocity", m_rightEncoder.getVelocity());
        SmartDashboard.putNumber("shooter/Velocity Requested", shooterSpeedReq);
    }
}
