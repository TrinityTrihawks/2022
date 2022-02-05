// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import static frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain subsystemInst = null;
    
    private final CANSparkMax frontLeftSparkMax = 
        new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
    private final CANSparkMax rearLeftSparkMax = 
        new CANSparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
    private final CANSparkMax frontRightSparkMax = 
        new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
    private final CANSparkMax rearRightSparkMax = 
        new CANSparkMax(DriveConstants.kBackRightMotorId, MotorType.kBrushless);
    
    private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftSparkMax, rearLeftSparkMax, frontRightSparkMax,rearRightSparkMax);
    
    private final RelativeEncoder frontLeftEncoder = 
        frontLeftSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder frontRightEncoder = 
        frontRightSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder backLeftEncoder = 
        rearLeftSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder backRightEncoder = 
        rearRightSparkMax.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);

    // The gyro sensor
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.kPigeonId);
    
    // Odometry class for tracking robot pose
    MecanumDriveOdometry mecanumOdometry = 
        new MecanumDriveOdometry(DriveConstants.kDriveKinematics, pigeon.getRotation2d());
    
    /**
     * Use this method to create a drivetrain instance. This method, in conjunction with a private constructor,
     * ensures that the
     * drivetrain class is a singleton, aka, that only one drivetrain object ever
     * gets created
     * 
     * 
     */
    public static Drivetrain getInstance() {
        if (subsystemInst == null) {
            return new Drivetrain();
        } else {
            return subsystemInst;
        }
    }

    /** Creates a new Drivetrain. */
    private Drivetrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        frontRightSparkMax.setInverted(true);
        rearRightSparkMax.setInverted(true);
        zeroHeading();
        frontLeftSparkMax.setIdleMode(IdleMode.kBrake);
        frontRightSparkMax.setIdleMode(IdleMode.kBrake);
        rearLeftSparkMax.setIdleMode(IdleMode.kBrake);
        rearRightSparkMax.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        updateOdometry();
        
        putMotorRPMToSmartDashboard();

        putGyroAngleToSmartDashboard();
    }

    private void updateOdometry() {
        mecanumOdometry.update(
                pigeon.getRotation2d(),
                getCurrentWheelSpeeds());
    }

    private void putMotorRPMToSmartDashboard() {
        SmartDashboard.putNumber("FLEnc (RPM)", frontLeftEncoder.getVelocity());
        SmartDashboard.putNumber("FREnc (RPM)", frontRightEncoder.getVelocity());
        SmartDashboard.putNumber("BLEnc (RPM)", backLeftEncoder.getVelocity());
        SmartDashboard.putNumber("BREnc (RPM)", backRightEncoder.getVelocity());
    }

    private void putGyroAngleToSmartDashboard() {
        SmartDashboard.putNumber("Gyro angle", getHeading());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * 
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
     *                      (sideways).
     * @param ySpeed        Speed of the robot in the y direction (forward/backwards).
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

    /** Sets the MotorControllers to a voltage. */
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

    // TODO: do we need to expose the encoders like this?
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
                frontLeftEncoder.getVelocity() / DriveConstants.kMotorRotationsPerMeter / 60, //rotations per minute * meters per rotation * minute per seconds
                backLeftEncoder.getVelocity() / DriveConstants.kMotorRotationsPerMeter / 60,
                frontRightEncoder.getVelocity() / DriveConstants.kMotorRotationsPerMeter / 60,
                backRightEncoder.getVelocity() / DriveConstants.kMotorRotationsPerMeter / 60);
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