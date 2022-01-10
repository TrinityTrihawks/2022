// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Drive Constants
     */
    public static final class DriveConstants {
        // TODO: Talon IDs
        public static final int kFrontLeftId = 0;
        public static final int kFrontRightId = 0;
        public static final int kBackLeftId = 0;
        public static final int kBackRightId = 0;

        // TODO: Polarity (reversed or not)
        // These should be either 1 to signify normal op or -1 for reversed
        public static final int kFLPolarity = 1;
        public static final int kFRPolarity = 1;
        public static final int kBLPolarity = 1;
        public static final int kBRPolarity = 1;

    }

    /**
     * Joystick Constants
     */
    public static final class JoystickConstants {
        public static final double kXDeadZone = 0.0; // Left-Right
        public static final double kYDeadZone = 0.0; // Front-Back
        public static final double kZDeadZone = 0.0; // Twist
    }

    /**
     * Network Constants (IP addreses and related info)
     */
    public static final class NetworkConstants {
        public static final String kFrontCameraIP = "10.42.15.00";
    }
}
