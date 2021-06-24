// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class LineFollow extends CommandBase {
  private Drivetrain drive;
  // Left and Right IR sensors
  private DigitalInput leftIR = new DigitalInput(12);
  private DigitalInput rightIR = new DigitalInput(11);
  private static boolean started = false;
  private long m_startTime;

  /** Creates a new LineFollow. */
  public LineFollow(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
    
  }

  public LineFollow() {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isleftIR = leftIR.get();
    boolean isrightIR = rightIR.get();
    
    // If the left IR sensor sees a black line, the romi will turn left
    if(isleftIR == true) {
      this.drive.tankDriveSpeed(0, 0.78);
      started = true;

    }
    // If the right IR sensor sees a black line, the romi will turn right
    if (isrightIR == true) {
      this.drive.tankDriveSpeed(0.59, 0);
      started = true;

      // If the left IR sensor sees a white line, the romi will move straight
    } else if (isleftIR == false && isrightIR == false) {
      this.drive.tankDriveSpeed(0.18, 0.15);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  // If both the left and right IR sensors see a black line, the romi will stop
  @Override
  public boolean isFinished() {
    boolean isleftIR = leftIR.get();
    boolean isrightIR = rightIR.get();
    boolean isFinished = false;
    if (System.currentTimeMillis() - m_startTime >= 25000) {
      if (started == true && isleftIR == true && isrightIR == true) {
        this.drive.tankDriveSpeed(0, 0);
      }
    } else if (System.currentTimeMillis() - m_startTime < 500000)
      this.drive.tankDriveSpeed(0.18, 0.15);
    
    return isFinished;
  }
}
