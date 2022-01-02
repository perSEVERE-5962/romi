// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FranticFetchAuto extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public FranticFetchAuto(Arm arm, Drivetrain drivetrain, ArrayList<Trajectory> trajectories) {

    addCommands(
      // execute the first path
      new MyRamseteCommand(trajectories.get(0),drivetrain),

      // get golf ball #1
      new RaiseArm(arm), // raise the arm before we move
      // new DriveDistance(0.4, 5, drivetrain), // drive forward at 60% speed for 10 inches
      // new LowerArm(arm), // lower the arm and collect the golf ball
      //new TurnTime(-0.4, 1, drivetrain),  // turn 180 degrees to the left at 30% speed
      //new TurnTime(0.4, 1, drivetrain),  // turn 180 degrees to the left at 30% speed
      //new Wait(5)

      // execute the second path
      new MyRamseteCommand(trajectories.get(1),drivetrain)

      // get golf ball #2
      // new DriveDistance(0.4, 4, drivetrain), // drive forward at 60% speed for 7 inches
      // new RaiseArm(arm), // raise the arm before we move
      // new DriveDistance(0.4, 3, drivetrain), // drive forward at 40% speed for 3 inches
      // new LowerArm(arm), // lower the arm and collect the golf ball
      // new TurnTime(-0.4, 1, drivetrain),  // turn 180 degrees to the left at 30% speed
      // new TurnTime(0.4, 1, drivetrain),  // turn 180 degrees to the left at 30% speed
     
      // // get golf ball #3
      // new DriveDistance(0.4, 4, drivetrain), // drive forward at 60% speed for 7 inches
      // new RaiseArm(arm), // raise the arm before we move
      // new DriveDistance(0.4, 3, drivetrain), // drive forward at 40% speed for 3 inches
      // new LowerArm(arm), // lower the arm and collect the golf ball
     
      // // turn around and go back to start
      // new TurnTime(-0.3, 5.5, drivetrain),  // turn 180 degrees to the left at 30% speed
      // new DriveDistance(0.4, 19, drivetrain) // drive forward at 60% speed for 10 inches

    );
  }

}
