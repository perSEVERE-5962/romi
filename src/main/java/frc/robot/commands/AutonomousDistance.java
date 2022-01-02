// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
      /**
       * Note that for DriveDistance and TurnDegrees the first
       * parameter is the speed.  For my Romi forward is a positive
       * value and backwards is a negative value.  It might be 
       * opposite for you.
       */
      //start
      new DriveDistance(0.6, 17, drivetrain), // drive forward at 60% speed for 17 inches
      new TurnDegrees(-0.3, 88, drivetrain),  // turn 88 degrees to the left at 30% speed
      new DriveDistance(0.6, 17, drivetrain), // drive forward at 60% speed for 17 inches

      // at yellow
      new TurnDegrees(-0.3, 90, drivetrain), // turn 90 degrees to the left at 30% speed

      // enter center
      new DriveDistance(0.5, 8, drivetrain), // drive forward at 50% speed for 8 inches
      new TurnDegrees(-0.3, 30, drivetrain), // turn 30 degrees to the left at 30% speed
      new DriveDistance(0.6, 12, drivetrain), // drive forward at 60% speed for 12 inches
      new TurnDegrees(0.3, 35, drivetrain), // turn 35 degrees to the right at 30% speed
      new DriveDistance(0.5, 6, drivetrain), // drive forward at 50% speed for 6 inches

      // in blue
      new TurnDegrees(0.3, 90, drivetrain), // turn 90 degrees to the right at 30% speed
      new DriveDistance(0.6, 15.5, drivetrain), // drive forward at 60% speed for 15.5 inches

      // in corner
      new TurnDegrees(0.3, 87, drivetrain), // turn 87 degrees to the right at 30% speed
      new DriveDistance(0.6, 15.5, drivetrain) // drive forward at 60% speed for 15.5 inches

      // done - in red
      );
  }
}
