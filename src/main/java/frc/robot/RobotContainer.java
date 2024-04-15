// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

//import org.photonvision.common.hardware.VisionLEDMode;
//import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ledSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */  

public class RobotContainer {

    // Create a robotcontainer to point at
    private static RobotContainer m_robot = null;

    public static void setRobot(RobotContainer robot){
        m_robot = robot;
    }

    public static RobotContainer getRobot(){
        return m_robot;
    }

    private final SendableChooser<Command> autoChooser;

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    //private final Vision m_vision = new Vision();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final ledSubsystem m_led = new ledSubsystem();

    // Define the controller being used
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    
    // Setup triggers
    Trigger xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    Trigger yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    Trigger aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    Trigger bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    Trigger startButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
    Trigger backButton = new JoystickButton(m_driverController, XboxController.Button.kBack.value);
    Trigger leftBumper = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    Trigger rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    Trigger leftStick = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
    Trigger rightStick = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
    Trigger povUp = new POVButton(m_driverController, 0);

    private double m_ampSpeed = ShooterConstants.kShooterAmpSpeed; // * Constants.VortexMotorConstants.kFreeSpeedRpm;
    private double m_speakerSpeed = ShooterConstants.kShooterSpeakerSpeed; // * Constants.VortexMotorConstants.kFreeSpeedRpm;
    private double m_passSpeed = ShooterConstants.kShooterPassSpeed; // * Constants.VortexMotorConstants.kFreeSpeedRpm;

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        // Register Named Commands

        // For legacy autos
        NamedCommands.registerCommand("IntakeON", new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));
        NamedCommands.registerCommand("intakeON", new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));
        NamedCommands.registerCommand("IntakeOFF", new InstantCommand(() -> m_shooter.intakeOFF()));
        NamedCommands.registerCommand("shooterON", new InstantCommand(() -> m_shooter.shooterON(SmartDashboard.getNumber("shooter/Shooter Speaker Speed", m_speakerSpeed))));
        NamedCommands.registerCommand("shooterOFF", new InstantCommand(() -> m_shooter.shooterOFF()));

        //NamedCommands.registerCommand("ResetGyro", new InstantCommand(() -> m_robot));
        // New autos use below
        NamedCommands.registerCommand("Shoot", shootSpeaker());
        NamedCommands.registerCommand("Pass", pass());
        
        // Add command to have limelight find note in auto
        // uses limelight
        NamedCommands.registerCommand("FindNote",
            (new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn("limelight")))
            .andThen (new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true, true, false)))
                    .until(m_shooter::getInvNoteSensor)
                    .withTimeout(1.25)
            .andThen(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")))
        );
        NamedCommands.registerCommand("FindSpeaker",
            (new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true, false, true)))
                    .until(m_shooter::tagged)
                    .withTimeout(5)
        );
        // Add commend to target in on thd april tag 
        /* 
        NamedCommands.registerCommand("TargetAmp", 
            (new RunCommand(() -> targetAmp()))
            .until(m_vision::targetAreaReached).withTimeout(3)
        );

        NamedCommands.registerCommand(("TargetSpeaker"),
            (new InstantCommand(() -> targetSpeaker()))
            .until(m_vision::targetAreaReached).withTimeout(1)
        );
        */
        NamedCommands.registerCommand("ShootAmp", shootAmp());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putNumber("shooter/Shooter Amp Speed", m_ampSpeed);
        SmartDashboard.putNumber("shooter/Shooter Speaker Speed", m_speakerSpeed);
        SmartDashboard.putNumber("shooter/Shooter Pass Speed", m_passSpeed);
        m_led.setColors(Color.kBlue, Color.kGreen); //Carrboro colors = 

        configureButtonBindings();
         
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true, rightStick.getAsBoolean(), leftStick.getAsBoolean()),
                m_robotDrive)
        );
    }

    private void configureButtonBindings() {

        // Shoot at the amp
        leftBumper.onTrue(shootAmp());

        // Shoot the shot at speaker
        rightBumper.onTrue(shootSpeaker());

        // Pass the note
        povUp.onTrue(pass());
 
        // Intake manual controls (a/b buttons)
        aButton.onTrue(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));    
        bButton.onTrue(new InstantCommand(() -> m_shooter.intakeOFF()));

        // Shooter manual controls (x/y buttons)
        xButton.onTrue(new InstantCommand(() -> m_shooter.shooterON(
            SmartDashboard.getNumber("shooter/Shooter Speaker Speed", m_speakerSpeed)
        )));

        yButton.onTrue(new InstantCommand(() -> m_shooter.shooterOFF()));

        // Start button fixes the odometry and resets to +90deg
        startButton.onTrue(Commands.runOnce(() -> m_robotDrive.fixHeading()));

        // Purge the shooter
        backButton.whileTrue(new InstantCommand(() -> m_shooter.shooterREV())
            .andThen(new InstantCommand(() -> m_shooter.intakeREV())))
            .onFalse(new InstantCommand(() -> m_shooter.shooterOFF())
        );

        // Use right stick for limelight activation
        rightStick.onTrue(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn("limelight")))
            .onFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight"))
        );

        // Left stick should head to april tags
        //xButton.whileTrue( new RunCommand(() -> m_robotDrive.drive(
        //            rpiRangeProp(),
         //           rpiAimProp(),
         //           -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
         //           false, false, false), m_robotDrive)
         //   .until(m_vision::targetAreaReached)
            //.onTrue(new InstantCommand(() -> m_vision.setLED(VisionLEDMode.kOn)))
            //.onFalse(new InstantCommand(() -> m_vision.setLED(VisionLEDMode.kOff))
        //);

        // Limit switch for detecting notes through intake
        // Returns true if no note seen
        Trigger noteSensor = new Trigger(() -> m_shooter.getNoteSensor());
        
        // Turn off intake and spool up shooter when note is sensed
        noteSensor.onFalse(noteSensed());
       
    }

    // Commands used for buttons and auto



    private Command shootAmp() {
        return (new InstantCommand(() -> m_shooter.shooterON(SmartDashboard.getNumber("shooter/Shooter Amp Speed", m_ampSpeed)))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeAmpSpeed))
            .andThen(new InstantCommand(() -> m_led.setRGB(255, 255, 255)))
            .andThen(new InstantCommand(() -> m_led.startBlinking()))
            .until(m_shooter::getInvshootSensor).withTimeout(1))
            .andThen(new WaitCommand(0.185))
            .andThen(new InstantCommand(() -> m_arm.rotateArmToAmp()))
            .andThen(new WaitCommand(.25))
            .andThen(new InstantCommand(() -> m_shooter.shooterOFF()))
            .andThen(new InstantCommand(()-> m_led.stopBlinking()))
            .andThen(new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen)))
            .andThen(new InstantCommand(() -> m_arm.rotateArmToBot()))
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed))));
    }   

    private Command pass() {
        return (new InstantCommand(() -> m_shooter.shooterON(SmartDashboard.getNumber("shooter/Shooter Pass Speed", m_passSpeed)))
                .until(m_shooter::isShooterReady).withTimeout(0.5)
//            .andThen(new InstantCommand(() -> m_arm.rotateArmToPass()))
//            .andThen(new WaitCommand(0.1))
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)))
                .until(m_shooter::getInvshootSensor).withTimeout(1)
            .andThen(new WaitCommand(0.1)) //0.25 for arm deploy
//            .andThen(new InstantCommand(() -> m_arm.rotateArmToBot()))
            .andThen(new InstantCommand(() -> m_shooter.shooterOFF())));
    }

    private Command noteSensed(){
        return (new InstantCommand(() -> m_shooter.intakeOFF())
            .alongWith(Commands.runOnce(() -> m_driverController.setRumble(RumbleType.kBothRumble, 2)))
            .alongWith(new InstantCommand(() -> m_led.setRGB(229, 30, 0)))
            .alongWith(new InstantCommand (() -> m_led.startBlinking()))
            .andThen(new InstantCommand(() -> m_shooter.shooterON(SmartDashboard.getNumber("shooter/Shooter Speaker Speed", m_speakerSpeed))))
            .andThen(new WaitCommand(0.5))
            .andThen(Commands.runOnce(() -> m_driverController.setRumble(RumbleType.kBothRumble, 0)))
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> m_led.stopBlinking()))
            .andThen(new InstantCommand(() -> m_led.setRGB(229, 30, 0)))
        );
    }

    private Command shootSpeaker() {
        return new SequentialCommandGroup(
        new InstantCommand(() -> m_shooter.shooterON(SmartDashboard.getNumber("shooter/Shooter Speaker Speed", m_speakerSpeed)))
            .until(m_shooter::isShooterReady).withTimeout(0.5)
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed))),
        new InstantCommand(() -> m_led.setRGB(255, 255, 255)),
        new InstantCommand(() -> m_led.startBlinking()),
        new WaitUntilCommand(() -> !m_shooter.shootSensor.get()).withTimeout(3),
        new WaitCommand(0.2), 
        new InstantCommand(() -> m_shooter.shooterOFF()),
        new InstantCommand(()-> m_led.stopBlinking()),
        new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen)));

    }
    /* 
    private Command targetAmp() {
        return new RunCommand(() -> m_robotDrive.drive(
                    rpiRangeProp(),
                    rpiAimProp(),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    false, false, false), m_robotDrive);
    }

    private Command targetSpeaker() {
        return new RunCommand(() -> m_robotDrive.drive(
                    rpiRangeProp(),
                    rpiAimProp(),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    false, false, false), m_robotDrive);
    }    
    
    // Aim to april tag if valid, otherwise use controller input

    private double rpiAimProp() {
        double kP = camera_rotation.getDouble(0.35); // 0.035
        if(m_vision.hasTarget()) {
            PhotonTrackedTarget target = m_vision.getTarget();
            if (m_vision.targetID(target) >= 4 && m_vision.targetID(target) <= 7){ 
                double targetAngularVel = m_vision.getYaw(target) * kP;
                targetAngularVel *= -1;
            SmartDashboard.putNumber("Camera Angular Vel", targetAngularVel);
                return targetAngularVel;
            }

        }
        
        return -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband);
    }
  
    private double rpiRangeProp() {
         double kP = camera_speed.getDouble(0.1); // 0.1
        if(m_vision.hasTarget()) {
            PhotonTrackedTarget target = m_vision.getTarget();
            if(m_vision.targetID(target) >= 4 && m_vision.targetID(target) <= 7){
                double targetForwardSpeed = 1/m_vision.getArea(target) * kP;
                targetForwardSpeed *= 1;
                SmartDashboard.putNumber("Camera Forward Speed", targetForwardSpeed);
                return targetForwardSpeed; 
            }
        }
        
        return -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband);
    }
    */
    // Get auto command from shuffleboard chooser

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    
}
