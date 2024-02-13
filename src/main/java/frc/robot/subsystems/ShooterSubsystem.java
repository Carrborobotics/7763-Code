// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    CANSparkMax ShooterRight = new CANSparkMax(Constants.ShooterConstants.kShooterRightId, MotorType.kBrushless);
    CANSparkMax ShooterLeft = new CANSparkMax(Constants.ShooterConstants.kShooterLeftId, MotorType.kBrushless);
    CANSparkMax intake1 = new CANSparkMax(Constants.ShooterConstants.kintake1Id, MotorType.kBrushless);
    CANSparkMax intake2 = new CANSparkMax(Constants.ShooterConstants.kintake2Id, MotorType.kBrushless);
    
    public ShooterSubsystem() {

    }

    public Command shooterFlip(){
        // Turn shooter on/off
        double sr = (ShooterRight.get() != 0) ? 0 : 1;
        double sl = (ShooterLeft.get() != 0) ? 0 : 1;
        ShooterRight.set(sr);
        ShooterLeft.set(sl);
        return null;
    }

    public Command intakeFlip(){
        // Turn intake on/off
        double i1 = (intake1.get() != 0) ? 0 : 1;
        double i2 = (intake2.get() != 0) ? 0 : 1;
        intake1.set(i1);
        intake2.set(i2);
        return null;
    }

    public Command shooterON(){
        ShooterRight.set(0.5);
        ShooterLeft.set(0.5);
        return null;
    }
    public Command intakeON(){
        intake1.set(0.5);
        intake2.set(-0.5);
        return null;
    }
  
    public Command intakeOFF(){
        intake1.set(0);
        intake2.set(0);
        return null;
    }

    public Command shooterOFF() {
        // ArmLeft.setNeutralMode(NeutralMode.Brake);
        ShooterRight.set(0);
        ShooterLeft.set(0);
        return null;
      }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

 

}
