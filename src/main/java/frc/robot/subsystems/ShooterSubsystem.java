// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /** Creates a new ShooterSubsystem. */
    //CANSparkMax ShooterRight = new CANSparkMax(Constants.ShooterConstants.kShooterRightId, MotorType.kBrushless);
    CANSparkMax ShooterLeft = new CANSparkMax(Constants.ShooterConstants.kShooterLeftId, MotorType.kBrushless);
    CANSparkMax intake1 = new CANSparkMax(Constants.ShooterConstants.kintake1Id, MotorType.kBrushless);
    CANSparkMax intake2 = new CANSparkMax(Constants.ShooterConstants.kintake2Id, MotorType.kBrushless);
    //CANSparkMax intake2 = new CANSparkMax(Constants.ShooterConstants.kintake2Id, MotorType.kBrushless);
    AnalogTrigger noteSensor = new AnalogTrigger(Constants.ShooterConstants.kNoteSensorId);

    //DigitalInput noteSensor = new DigitalInput(Constants.ShooterConstants.kNoteSensorId);
    
    public ShooterSubsystem() {
        noteSensor.setLimitsVoltage(1.5, 4);
    }

    public Command shooterFlip(){
        // Turn shooter on/off
        if(ShooterLeft.get() > 0.1 | ShooterLeft.get() < 0.1) {
            ShooterLeft.set(0);
            // ShooterRight.set(0);
        }
        else {
            ShooterLeft.set(0.5);
            // ShooterRight.set(0.5);
        }
        //if(ShooterRight.get() > 0.1 | ShooterRight.get() < 0.1) {
        //    ShooterRight.set(0);
        //}
        //else {
        //    ShooterRight.set(0.5);
        //}
        return null;
    }

    public Command shooterON(){
        return runOnce(()-> ShooterLeft.set(0.2));
    }

    public Command shooterOFF() {
        return runOnce(()-> ShooterLeft.set(0));
    }

    public Command intakeON(){
        intake1.set(1);
        intake2.set(-1);
        return null;
    }
  
    public Command intakeOFF(){
        intake1.set(0);
        intake2.set(0);
        return null;
    }
    
    public Command intakeREV(){
        intake1.set(-0.1);
        intake2.set(0.5);
        return null;
    }


    @Override
    public void periodic() {
        // Handle checking the note sensor to see if the intake is loaded
        // noteSensor.get() is True if there is not a note
//         System.out.println(noteSensor.getTriggerState());
//         if(!noteSensor.getTriggerState()){
//             intakeOFF();
//         }
//         else{
//             intakeON();
//         }
   }
}
