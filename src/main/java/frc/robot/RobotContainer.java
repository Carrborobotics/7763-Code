// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Limelight;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
    private final SendableChooser<Command> autoChooser;
    private final Limelight m_vision = new Limelight("limelight");

    // Define the controller being used
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Register Named Commands
        NamedCommands.registerCommand("IntakeON", shootersubsystem.intakeON());
        NamedCommands.registerCommand("IntakeOFF", shootersubsystem.intakeOFF());
        NamedCommands.registerCommand("shooterON", shootersubsystem.shooterON());
        NamedCommands.registerCommand("shooterOFF", shootersubsystem.shooterOFF());

        configureButtonBindings();
        configureAuto();
        m_vision.getName();

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

    private void configureAuto() {

    }
    
    private void configureButtonBindings() {
        //A button: puts the wheels to X
        new JoystickButton(m_driverController, Button.kA.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.setX(),
                m_robotDrive));

        // Left Bumper: Turn shooter on/off
        new JoystickButton(m_driverController, Button.kLeftBumper.value)
            .onTrue(new RunCommand(()-> shootersubsystem.shooterFlip()));

        // Right Bumper: Turn intake on/off
        new JoystickButton(m_driverController, Button.kRightBumper.value)
            .onTrue(new RunCommand(()-> shootersubsystem.intakeFlip()));
    
        SmartDashboard.putData("Two Piece Auto Test", new PathPlannerAuto("2 Piece Auto"));
        SmartDashboard.putData("Shooter Test", new PathPlannerAuto("shooter test"));
        SmartDashboard.putData("Square Dance", new PathPlannerAuto("Square Auto"));

        return;
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
