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
        //start
     new DriveDistance(0.6, 16.5, drivetrain), // drive forward at 60% speed for 16.5 inches
     new TurnDegrees(-0.3, 80, drivetrain),  // turn 80 degrees to the left at 30% speed
     new DriveDistance(0.6, 16.75, drivetrain), // drive forward at 60% speed for 16.75 inches
 
     // at yellow
     new TurnDegrees(-0.3, 87, drivetrain), // turn 87 degrees to the left at 30% speed
 
     // enter center
     new DriveDistance(0.5, 7, drivetrain), // drive forward at 50% speed for 7 inches
     new TurnDegrees(-0.3, 30, drivetrain), // turn 30 degrees to the left at 30% speed
     new DriveDistance(0.5, 9, drivetrain), // drive forward at 50% speed for 9 inches
     new TurnDegrees(0.3, 60, drivetrain), // turn 60 degrees to the right at 30% speed
     new DriveDistance(0.5, 9.75, drivetrain), // drive forward at 50% speed for 9.75 inches
 
     // in blue
     new TurnDegrees(0.3, 90, drivetrain), // turn 90 degrees to the right at 30% speed
     new DriveDistance(0.6, 16.5, drivetrain), // drive forward at 60% speed for 16.5 inches
 
     // in corner
     new TurnDegrees(0.3, 90, drivetrain), // turn 90 degrees to the right at 30% speed
     new DriveDistance(0.6, 15.5, drivetrain)); // drive forward at 60% speed for 15.5 inches
 
     // done - in red

    
  }
}
