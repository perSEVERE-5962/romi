// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Joystick {
        // Button mapping for a Logitech gamepad.
        public static final int X = 3;
        public static final int A = 1;
        public static final int B = 2;
        public static final int Y = 4;
    }

    public final class Arm {
        // Port configuration to match physical configuration on
        // Romi board as well as configuration on http://wpilib.local
        public static final int LIFT_PORT = 3;
        public static final int TILT_PORT = 4;

        public static final double LIFT_UP_ANGLE = 60;
        public static final double LIFT_DOWN_ANGLE = 120;
        public static final double TILT_IN_ANGLE = 70;
        public static final double TILT_OUT_ANGLE = 120;
    }

    public static final class DriveConstants {
        public static final double ksVolts = 0.929;
        public static final double kvVoltSecondsPerMeter = 6.33;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

        public static final double kPDriveVel = 0.085;

        public static final double kTrackwidthMeters = 0.142072613;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

}
