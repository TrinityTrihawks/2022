// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain subsystemInst = null;

    /**
     * Use this method to create a drivetrain instance. This method ensures that the
     * drivetrain class is a singleton, aka, that only one drivetrain object ever
     * gets created
     * 
     * @return
     */
    public static Drivetrain getInstance() {
        if (subsystemInst == null) {
            return new Drivetrain();
        } else {
            return subsystemInst;
        }
    }

    private final CANSparkMax frontLeftSparkMax = new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private final CANSparkMax rearLeftSparkMax = new CANSparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
    private final CANSparkMax frontRightSparkMax = new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final CANSparkMax rearRightSparkMax = new CANSparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);

    private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftSparkMax, rearLeftSparkMax, frontRightSparkMax,rearRightSparkMax);

    private final RelativeEncoder frontLeftEncoder = frontLeftSparkMax.getEncoder();
    private final RelativeEncoder frontRightEncoder = frontRightSparkMax.getEncoder();
    private final RelativeEncoder backLeftEncoder = rearLeftSparkMax.getEncoder();
    private final RelativeEncoder backRightEncoder = rearRightSparkMax.getEncoder();
    // The gyro sensor
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.kPigeonId);

    // Odometry class for tracking robot pose
    MecanumDriveOdometry mecanumOdometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, pigeon.getRotation2d());

    /** Creates a new Drivetrain. */
    public Drivetrain() {
        // Sets the distance per pulse for the encoders
        // frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // backLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // backRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        frontRightSparkMax.setInverted(true);
        rearRightSparkMax.setInverted(true);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        mecanumOdometry.update(
                pigeon.getRotation2d(),
                new MecanumDriveWheelSpeeds(
                        frontLeftEncoder.getVelocity(),
                        backLeftEncoder.getVelocity(),
                        frontRightEncoder.getVelocity(),
                        backRightEncoder.getVelocity()));
        SmartDashboard.putNumber("FLEnc", frontLeftEncoder.getVelocity());
        SmartDashboard.putNumber("FREnc", frontRightEncoder.getVelocity());
        SmartDashboard.putNumber("BLEnc", backLeftEncoder.getVelocity());
        SmartDashboard.putNumber("BREnc", backRightEncoder.getVelocity());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return mecanumOdometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        mecanumOdometry.resetPosition(pose, pigeon.getRotation2d());
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            mecanumDrive.driveCartesian(ySpeed, xSpeed, rot, -pigeon.getAngle());
        } else {
            mecanumDrive.driveCartesian(ySpeed, xSpeed, rot);
        }
    }

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        frontLeftSparkMax.setVoltage(volts.frontLeftVoltage);
        rearLeftSparkMax.setVoltage(volts.rearLeftVoltage);
        frontRightSparkMax.setVoltage(volts.frontRightVoltage);
        rearRightSparkMax.setVoltage(volts.rearRightVoltage);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftEncoder.setPosition(0);
        backLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        backRightEncoder.setPosition(0);
    }

    /**
     * Gets the front left drive encoder.
     *
     * @return the front left drive encoder
     */
    public RelativeEncoder getFrontLeftEncoder() {
        return frontLeftEncoder;
    }

    /**
     * Gets the rear left drive encoder.
     *
     * @return the rear left drive encoder
     */
    public RelativeEncoder getRearLeftEncoder() {
        return backLeftEncoder;
    }

    /**
     * Gets the front right drive encoder.
     *
     * @return the front right drive encoder
     */
    public RelativeEncoder getFrontRightEncoder() {
        return frontRightEncoder;
    }

    /**
     * Gets the rear right drive encoder.
     *
     * @return the rear right encoder
     */
    public RelativeEncoder getRearRightEncoder() {
        return backRightEncoder;
    }

    /**
     * Gets the current wheel speeds.
     *
     * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
     */
    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeftEncoder.getVelocity(),
                backLeftEncoder.getVelocity(),
                frontRightEncoder.getVelocity(),
                backRightEncoder.getVelocity());
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        mecanumDrive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        pigeon.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return pigeon.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -pigeon.getRate();
    }
}