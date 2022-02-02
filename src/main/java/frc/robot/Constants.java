// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

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
        // Spark IDs
        public static final int kFrontLeftMotorId = 11;
        public static final int kFrontRightMotorId = 12;
        public static final int kBackLeftMotorId = 13;
        public static final int kBackRightMotorId = 14;

        public static final int kPigeonId = 15;

        // vvv From https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/mecanumcontrollercommand/Constants.java

        // Encoder IDs
        public static final int[] kFrontLeftEncoderIds = new int[] {0, 1};
        public static final int[] kBackLeftEncoderIds = new int[] {2, 3};
        public static final int[] kFrontRightEncoderIds = new int[] {4, 5};
        public static final int[] kBackRightEncoderIds = new int[] {6, 7};
    
        // Encoder direction
        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kBackLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kBackRightEncoderReversed = true;
        
        // Distance between centers of right and left wheels on robot    
        public static final double kTrackWidth = 0.56; // 22in
        // Distance between centers of front and back wheels on robot
        public static final double kWheelBase = 0.5; // 20in
    
        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));   // TODO: "The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be used for any other purpose."
        
        public static final double kGearRatio = 10.71; //10.71:1
        public static final int kEncoderCPR = 42; //counts per revolution
        public static final double kWheelDiameterMeters = 0.1524; // 6in
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR * kGearRatio); 
        public static final double kMotorRotationsPerMeter = 
            (kWheelDiameterMeters * Math.PI) / kGearRatio;

    }

    /**
     * Joystick Constants
     */
    public static final class JoystickConstants {

        public static final int kMainJoystickPort = 0;
        public static final int kAuxJoystickPort  = 4;

        public static final double kXDeadZone = 0.1; // Left-Right
        public static final double kYDeadZone = 0.1; // Front-Back
        public static final double kTwistDeadZone = 0.1; // Twist

        public static final double kStaticThrottleScalar = 0.3; // multiply inputs' values by this

        public static final int kZeroButtonId = 7;
        public static final int kSwitchDriveModeButtonId = 9;

    }

    /**
     * Network Constants (IP addreses and related info)
     */
    public static final class NetworkConstants {
        public static final String kFrontCameraIP = "10.42.15.00";
    }
}
