// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
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
    private final Limelight m_vision = new Limelight("limelight");

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

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        // Register Named Commands

        // For legacy autos
        NamedCommands.registerCommand("IntakeON", new InstantCommand(() -> m_shooter.intakeON()));
        NamedCommands.registerCommand("intakeON", new InstantCommand(() -> m_shooter.intakeON()));
        NamedCommands.registerCommand("IntakeOFF", new InstantCommand(() -> m_shooter.intakeOFF()));
        NamedCommands.registerCommand("shooterON", new InstantCommand(() -> m_shooter.shooterON()));
        NamedCommands.registerCommand("shooterOFF", new InstantCommand(() -> m_shooter.shooterOFF()));
        // For new autos
        NamedCommands.registerCommand("Led On", new InstantCommand(() -> m_vision.ledOn()));
        NamedCommands.registerCommand("Led Off", new InstantCommand(() -> m_vision.ledOff()));
        NamedCommands.registerCommand("Take Snap", new InstantCommand(() -> m_vision.takeSnap()));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureButtonBindings();
        m_vision.setPipeline(0);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));
        
    }
    
    private void configureButtonBindings() {

        // Purge the shooter
        leftBumper.whileTrue(new InstantCommand(() -> m_shooter.shooterREV()))
            .onFalse(new InstantCommand(() -> m_shooter.shooterOFF())
        );

        // Shoot the shot
        rightBumper.onTrue(Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_shooter.intakeON()),
                new InstantCommand(() -> m_shooter.shooterON())
            ).withTimeout(2),
                new InstantCommand(() -> m_shooter.shooterOFF())));

        // Intake manual controls (a/b buttons)
        aButton.onTrue(new InstantCommand(() -> m_shooter.intakeON()));    
        bButton.onTrue(new InstantCommand(() -> m_shooter.intakeOFF()));

        // Shooter manual controls (x/y buttons)
        xButton.onTrue(new InstantCommand(() -> m_shooter.shooterON()));
        yButton.onTrue(new InstantCommand(() -> m_shooter.shooterOFF()));

        // Start button fixes the odometry and resets to +90deg
        startButton.onTrue(Commands.runOnce(() -> m_robotDrive.fixHeading()));

        // Limit switch for detecting notes through intake
        // Returns true if no note seen
        Trigger noteSensor = new Trigger(() -> m_shooter.getNoteSensor());
        
        // Turn off intake and spool up shooter when note is sensed
        noteSensor.onFalse(Commands.parallel(
            new InstantCommand(() -> m_shooter.intakeOFF()),
            new InstantCommand(() -> m_shooter.shooterON())));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
