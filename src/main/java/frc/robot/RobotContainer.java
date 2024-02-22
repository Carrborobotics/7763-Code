// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot; 

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final SendableChooser<Command> autoChooser;

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
    private final Limelight m_vision = new Limelight("limelight");

    // Define the controller being used
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    // The container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        System.out.println("Deploy Dir" + Filesystem.getDeployDirectory());
        // Register Named Commands
        NamedCommands.registerCommand("Turn Intake On", shootersubsystem.intakeON());
        NamedCommands.registerCommand("Turn Intake Off", shootersubsystem.intakeOFF());
        NamedCommands.registerCommand("Turn Shooter On", shootersubsystem.shooterON());
        NamedCommands.registerCommand("Turn Shooter Off", shootersubsystem.shooterOFF());
        NamedCommands.registerCommand("Led On", m_vision.ledOn());
        NamedCommands.registerCommand("Led Off", m_vision.ledOff());
        NamedCommands.registerCommand("Take Snap", m_vision.takeSnap());
        
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
        //A button: puts the wheels to X
        new JoystickButton(m_driverController, Button.kStart.value)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

        // Right Bumper: Turn intake on/off
        new JoystickButton(m_driverController, Button.kRightBumper.value)
            .whileTrue(shootersubsystem.intakeON());

        new JoystickButton(m_driverController, Button.kY.value)
            .onTrue(shootersubsystem.intakeREV());
    
        new JoystickButton(m_driverController, Button.kX.value)
            .onTrue(shootersubsystem.intakeOFF());
        
        new JoystickButton(m_driverController, Button.kA.value)
            .onTrue(m_vision.ledOn());
        
        new JoystickButton(m_driverController, Button.kB.value)
            .onTrue(m_vision.ledOff());

        new JoystickButton(m_driverController, Button.kBack.value)
            .onTrue(m_vision.takeSnap());

        new JoystickButton(m_driverController, Button.kRightStick.value)
            .onTrue(shootersubsystem.shooterON());

        new JoystickButton(m_driverController, Button.kLeftStick.value)
            .onTrue(shootersubsystem.shooterOFF());
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
