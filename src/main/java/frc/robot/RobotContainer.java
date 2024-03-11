// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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

    private double m_ampSpeed = ShooterConstants.kShooterAmpSpeed;
    private double m_speakerSpeed = ShooterConstants.kShooterSpeakerSpeed;

    private ShuffleboardTab tabSelected = Shuffleboard.getTab("tweaks");

    private GenericEntry amp_speed = tabSelected
        .add("Amp Speed Scale", m_ampSpeed)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    
    private GenericEntry speaker_speed = tabSelected
        .add("Speaker Speed Scaler", m_speakerSpeed)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    
    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        PhotonCamera camera = new PhotonCamera("aprilcam");

        // Register Named Commands

        // For legacy autos
        NamedCommands.registerCommand("IntakeON", new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));
        NamedCommands.registerCommand("intakeON", new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));
        NamedCommands.registerCommand("IntakeOFF", new InstantCommand(() -> m_shooter.intakeOFF()));
        NamedCommands.registerCommand("shooterON", new InstantCommand(() -> m_shooter.shooterON(speaker_speed.getDouble(1))));
        NamedCommands.registerCommand("shooterOFF", new InstantCommand(() -> m_shooter.shooterOFF()));

        // New autos use below
        NamedCommands.registerCommand("Shoot", shootSpeaker());
        
        // Add command to have limelight find note in auto
        // uses limelight
        NamedCommands.registerCommand("FindNote",
            (new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn("limelight")))
            .andThen (new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true, true)))
                    .until(m_shooter::getInvNoteSensor)
                    .withTimeout(.75)
            .andThen(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")))
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("Has AprilTag", camera.getLatestResult().hasTargets());

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            SmartDashboard.putNumber("April Yaw", target.getYaw());
            SmartDashboard.putNumber("April Area", target.getArea());
            SmartDashboard.putNumber("April Skew", target.getSkew());
        }
        configureButtonBindings();
         
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true, rightStick.getAsBoolean()),
                m_robotDrive)
        );
    }

    private void configureButtonBindings() {

        // Shoot at the amp
        leftBumper.onTrue(shootAmp());

        // Shoot the shot at speaker
        rightBumper.onTrue(shootSpeaker());
 
        // Intake manual controls (a/b buttons)
        aButton.onTrue(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed)));    
        bButton.onTrue(new InstantCommand(() -> m_shooter.intakeOFF()));

        // Shooter manual controls (x/y buttons)
        xButton.onTrue(new InstantCommand(() -> m_shooter.shooterON(speaker_speed.getDouble(1))));
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
        .onFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")));

        // Limit switch for detecting notes through intake
        // Returns true if no note seen
        Trigger noteSensor = new Trigger(() -> m_shooter.getNoteSensor());
        
        // Turn off intake and spool up shooter when note is sensed
        noteSensor.onFalse(noteSensed());
    }
    private Command shootAmp() {
        return (new InstantCommand(() -> m_shooter.shooterON(amp_speed.getDouble(1))))
            .andThen(new WaitCommand(1))
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeAmpSpeed)))
            .andThen(new WaitCommand(1.5))
            .andThen(new InstantCommand(() -> m_shooter.shooterOFF()))
            .andThen(new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed))
        );
    }

    private Command noteSensed(){
        return (new InstantCommand(() -> m_shooter.intakeOFF())
            .andThen(new InstantCommand(() -> m_shooter.shooterON(speaker_speed.getDouble(1))))
        );
    }

    private Command shootSpeaker() {
        return new InstantCommand(() -> m_shooter.intakeON(ShooterConstants.kIntakeSpeakerSpeed))
            .andThen(new InstantCommand(() -> m_shooter.shooterON(speaker_speed.getDouble(1))))
            .andThen(new WaitCommand(0.1))
            .andThen(new InstantCommand(() -> m_shooter.shooterOFF())
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    
}
