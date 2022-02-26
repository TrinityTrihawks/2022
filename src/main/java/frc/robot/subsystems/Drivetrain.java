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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain subsystemInst = null;

    private final CANSparkMax frontLeftSparkMax = new CANSparkMax(DriveConstants.kFrontLeftMotorId,
            MotorType.kBrushless);
    private final CANSparkMax rearLeftSparkMax = new CANSparkMax(DriveConstants.kBackLeftMotorId, MotorType.kBrushless);
    private final CANSparkMax frontRightSparkMax = new CANSparkMax(DriveConstants.kFrontRightMotorId,
            MotorType.kBrushless);
    private final CANSparkMax rearRightSparkMax = new CANSparkMax(DriveConstants.kBackRightMotorId,
            MotorType.kBrushless);

    private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftSparkMax, rearLeftSparkMax, frontRightSparkMax,
            rearRightSparkMax);

    private final RelativeEncoder frontLeftEncoder = frontLeftSparkMax
            .getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder frontRightEncoder = frontRightSparkMax
            .getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder backLeftEncoder = rearLeftSparkMax
            .getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);
    private final RelativeEncoder backRightEncoder = rearRightSparkMax
            .getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, DriveConstants.kEncoderCPR);

    // The gyro sensor
    private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.kPigeonId);

    // Odometry class for tracking robot pose
    private MecanumDriveOdometry mecanumOdometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics,
            pigeon.getRotation2d());

    private SlewRateLimiter ylimiter = new SlewRateLimiter(DriveConstants.kSlewValue);
    private SlewRateLimiter xlimiter = new SlewRateLimiter(DriveConstants.kSlewValue);
    private SlewRateLimiter zlimiter = new SlewRateLimiter(DriveConstants.kSlewValue);

    private PIDController frController = new PIDController(DriveConstants.kfrP, DriveConstants.kfrI, 0);
    private PIDController flController = new PIDController(DriveConstants.kflP, DriveConstants.kflI, 0);
    private PIDController brController = new PIDController(DriveConstants.kbrP, DriveConstants.kbrI, 0);
    private PIDController blController = new PIDController(DriveConstants.kblP, DriveConstants.kblI, 0);

    /**
     * Use this method to create a drivetrain instance. This method, in conjunction
     * with a private constructor,
     * ensures that the
     * drivetrain class is a singleton, aka, that only one drivetrain object ever
     * gets created
     * 
     * 
     */
    public static Drivetrain getInstance() {
        if (subsystemInst == null) {
            subsystemInst = new Drivetrain();
        }
        return subsystemInst;
    }

    /** Creates a new Drivetrain. */
    private Drivetrain() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        frontRightSparkMax.setInverted(true);
        rearRightSparkMax.setInverted(true);

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
        putWheelSpeedsToSmartDashboard();
        getPIDConstantsFromSmartDashboard();
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
    
    private void putWheelSpeedsToSmartDashboard() {
        MecanumDriveWheelSpeeds spds = getCurrentWheelSpeeds();
        SmartDashboard.putNumber("FRWheel m/s", spds.frontRightMetersPerSecond);
        SmartDashboard.putNumber("FLWheel m/s", spds.frontLeftMetersPerSecond);
        SmartDashboard.putNumber("BRWheel m/s", spds.rearRightMetersPerSecond);
        SmartDashboard.putNumber("BLWheel m/s", spds.rearLeftMetersPerSecond);
    }

    private void getPIDConstantsFromSmartDashboard() {

        frController.setPID(SmartDashboard.getNumber("frP", 0),
                            SmartDashboard.getNumber("frI", 0),
                            0);

        flController.setPID(SmartDashboard.getNumber("flP", 0),
                            SmartDashboard.getNumber("flI", 0),
                            0);

        brController.setPID(SmartDashboard.getNumber("brP", 0),
                            SmartDashboard.getNumber("brI", 0),
                            0);

        blController.setPID(SmartDashboard.getNumber("blP", 0),
                            SmartDashboard.getNumber("blI", 0),
                            0);

    }

    /**
     * Returns the currently-estimated pose of the robot.
     * 
     */
    public Pose2d getPose() {
        Pose2d mecPose = mecanumOdometry.getPoseMeters();
        return new Pose2d(mecPose.getX(), -mecPose.getY(), mecPose.getRotation());
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
     * Drives the robot at given x, y and theta intendedSpeeds. Speeds range from
     * [-1, 1]
     * and the linear
     * intendedSpeeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (sideways).
     * @param ySpeed        Speed of the robot in the y direction
     *                      (forward/backwards).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y intendedSpeeds are relative
     *                      to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double ySlewSpeed = ylimiter.calculate(ySpeed);
        double xSlewSpeed = xlimiter.calculate(xSpeed);
        double rotSlew = zlimiter.calculate(rot);

        if (fieldRelative) {
            mecanumDrive.driveCartesian(xSlewSpeed, ySlewSpeed, rotSlew, pigeon.getAngle());
        } else {
            mecanumDrive.driveCartesian(xSlewSpeed, ySlewSpeed, rotSlew);
        }
    }

    public void drive(MecanumDriveWheelSpeeds intendedSpeeds) {
        // TODO: add feedforward
        // final double frontLeftFeedforward =
        // m_feedforward.calculate(intendedSpeeds.frontLeftMetersPerSecond);
        // final double frontRightFeedforward =
        // m_feedforward.calculate(intendedSpeeds.frontRightMetersPerSecond);
        // final double backLeftFeedforward =
        // m_feedforward.calculate(intendedSpeeds.rearLeftMetersPerSecond);
        // final double backRightFeedforward =
        // m_feedforward.calculate(intendedSpeeds.rearRightMetersPerSecond);

        MecanumDriveWheelSpeeds actualSpeeds = getCurrentWheelSpeeds();

        final double frontLeftOutput = flController.calculate(actualSpeeds.frontLeftMetersPerSecond,
                intendedSpeeds.frontLeftMetersPerSecond);

        final double frontRightOutput = frController.calculate(actualSpeeds.frontRightMetersPerSecond,
                intendedSpeeds.frontRightMetersPerSecond);

        final double backLeftOutput = blController.calculate(actualSpeeds.rearLeftMetersPerSecond,
                intendedSpeeds.rearLeftMetersPerSecond);

        final double backRightOutput = brController.calculate(actualSpeeds.rearRightMetersPerSecond,
                intendedSpeeds.rearRightMetersPerSecond);

        setDriveMotorControllersVolts(new MecanumDriveMotorVoltages(frontLeftOutput,
                frontRightOutput,
                backLeftOutput,
                backRightOutput));
    }

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

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeftEncoder.getVelocity() * DriveConstants.kMetersPerMotorRotation / 60, // rotations per minute *
                                                                                              // meters per rotation *
                                                                                              // minute per seconds
                frontRightEncoder.getVelocity() * DriveConstants.kMetersPerMotorRotation / 60,
                backLeftEncoder.getVelocity() * DriveConstants.kMetersPerMotorRotation / 60,
                backRightEncoder.getVelocity() * DriveConstants.kMetersPerMotorRotation / 60);
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

    public void zeroHeading() {
        pigeon.reset();
    }

    public double getHeading() {
        return pigeon.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return pigeon.getRate();
    }

    public void brake() {
        frontLeftSparkMax.setIdleMode(IdleMode.kBrake);
        frontRightSparkMax.setIdleMode(IdleMode.kBrake);
        rearLeftSparkMax.setIdleMode(IdleMode.kBrake);
        rearRightSparkMax.setIdleMode(IdleMode.kBrake);
    }

    public void releaseBrake() {
        frontLeftSparkMax.setIdleMode(IdleMode.kCoast);
        frontRightSparkMax.setIdleMode(IdleMode.kCoast);
        rearLeftSparkMax.setIdleMode(IdleMode.kCoast);
        rearRightSparkMax.setIdleMode(IdleMode.kCoast);
    }

}