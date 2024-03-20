// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax shooterLeft;
    private CANSparkFlex shooterRight;
    private CANSparkMax intake1;
    private CANSparkMax intake2;
    private DigitalInput noteSensor;


      public ShooterSubsystem() {

        shooterLeft = new CANSparkMax(Constants.ShooterConstants.kShooterLeftId, MotorType.kBrushless);
        shooterRight = new CANSparkFlex(Constants.ShooterConstants.kShooterRightId, MotorType.kBrushless);
        shooterLeft.enableVoltageCompensation(12);
        shooterRight.enableVoltageCompensation(12);
        // shooterRight.setOpenLoopRampRate(0.5);
        intake1 = new CANSparkMax(Constants.ShooterConstants.kintake1Id, MotorType.kBrushless);
        intake2 = new CANSparkMax(Constants.ShooterConstants.kintake2Id, MotorType.kBrushless);
        noteSensor = new DigitalInput(Constants.ShooterConstants.kNoteSensorId);

        // Start up with the intake ON
        //intakeON(1);
    }
    // should be 0.125 for amp and 1 for speaker 
    public void shooterON(double inputSpeed){
        shooterLeft.set(inputSpeed);
        shooterRight.set(inputSpeed);
    }

    public void shooterREV(){
        shooterLeft.set(-1);
        shooterRight.set(1);
    }

    public void shooterOFF() {
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
        SmartDashboard.putBoolean("Note Sensor", noteSensor.get());
        SmartDashboard.putNumber("Shooter Left Voltage", shooterLeft.getBusVoltage());
        SmartDashboard.putNumber("Shooter Right Voltage", shooterRight.getBusVoltage());
    }
}
