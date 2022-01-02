// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickArmCommand extends CommandBase {

  private final Arm m_arm;
  private final XboxController m_joystick;

  /* Creates a new command which controls the arm via 
   * a Joystick.
   * 
   * @param arm Arm subsystem
   * @param joystick Joystick to be used to control the arm
   * 
   */
  public JoystickArmCommand(Arm arm, XboxController joystick) {
    m_arm = arm;
    m_joystick = joystick;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_joystick.getRawButton(Constants.Joystick.Y)) {
      // move arm up, but first position scoop in middle
      m_arm.setTiltAngle(120);
      m_arm.setLiftAngle(60);
    }

    if (m_joystick.getRawButton(Constants.Joystick.A)) {
      // move arm down, but first position scoop in middle
      m_arm.setTiltAngle(120);
      m_arm.setLiftAngle(120);
    }

    if (m_joystick.getRawButton(Constants.Joystick.X)) {
      // close scoop
      m_arm.setTiltAngle(70);
    }

    if (m_joystick.getRawButton(Constants.Joystick.B)) {
      //open scoop
      m_arm.setTiltAngle(120);
    }
    SmartDashboard.putNumber("Lift Position", m_arm.getLiftPos());
    SmartDashboard.putNumber("Lift Angle", m_arm.getLiftAngle());
    SmartDashboard.putNumber("Tilt Position", m_arm.getTiltPos());
    SmartDashboard.putNumber("Tile Angle", m_arm.getTiltAngle());

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
