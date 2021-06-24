// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.FranticFetchAuto;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.JoystickArmCommand;
import frc.robot.subsystems.Arm;

// import frc.robot.commands.AutonomousDistance;
// import frc.robot.commands.AutonomousTime;

// import java.io.IOException;
// import java.nio.file.Path;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  //private final Arm m_arm = new Arm();

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Trajectory path1Trajectory;
  private Trajectory path2Trajectory;
  private final int numberOfPaths = 2;
  private ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

  private static final String path1TrajectoryJSON = "paths/path1.wpilib.json";
  private static final String path2TrajectoryJSON = "paths/path2.wpilib.json";


  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    path1Trajectory = new Trajectory();
    try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(path1TrajectoryJSON);
      path1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
    } catch (IOException ex) {
      System.out.println("Unable to open trajectory: " + path1TrajectoryJSON + "  " + ex.getStackTrace().toString());
      DriverStation.reportError("Unable to open trajectory: " + path1TrajectoryJSON, ex.getStackTrace());
    }
    trajectories.add(path1Trajectory);

    path2Trajectory = new Trajectory();
    try {
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(path2TrajectoryJSON);
      path2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    } catch (IOException ex) {
      System.out.println("Unable to open trajectory: " + path2TrajectoryJSON + "  " + ex.getStackTrace().toString());
      DriverStation.reportError("Unable to open trajectory: " + path2TrajectoryJSON, ex.getStackTrace());
    }
    trajectories.add(path2Trajectory);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //m_arm.setDefaultCommand( new JoystickArmCommand(m_arm, m_controller));

    m_drivetrain.setDefaultCommand( new ArcadeDrive(m_drivetrain,
    () -> -m_controller.getRawAxis(1),
    () -> m_controller.getRawAxis(4)
    ));

    // // Example of how to use the onboard IO
    // Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    // onboardButtonA
    //     .whenActive(new PrintCommand("Button A Pressed"))
    //     .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    // m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand());
    // m_chooser.addOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    // m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    // SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonomousDistance(m_drivetrain);
    //return new FranticFetchAuto(m_arm, m_drivetrain, trajectories);
  }

  public Joystick getJoystick() {
    return m_controller;
  }

}
