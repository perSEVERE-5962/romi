package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Servo m_lift = new Servo(Constants.Arm.LIFT_PORT);
  private double m_liftPos;

  private final Servo m_tilt = new Servo(Constants.Arm.TILT_PORT);
  private double m_tiltPos;

  // Creates a new Arm subsystem
  public Arm() {
    reset();
  }

  // Reset position to resting state
  public void reset() {
    m_liftPos = 0.5;
    m_lift.set(m_liftPos);

    m_tiltPos = 0.5;
    m_tilt.set(m_tiltPos);
  }

  public double getLiftPos() {
    return m_lift.getPosition();
  }

  public double getLiftAngle() {
    return m_lift.getAngle();
  }

  public void setLiftAngle(double degrees) {
    m_lift.setAngle(degrees);
  }

  public double getTiltPos() {
    return m_tilt.getPosition();
  }

  public double getTiltAngle() {
    return m_tilt.getAngle();
  }

  public void setTiltAngle(double degrees) {
    m_tilt.setAngle(degrees);
  }

  @Override
  public void periodic() {
  }
}
