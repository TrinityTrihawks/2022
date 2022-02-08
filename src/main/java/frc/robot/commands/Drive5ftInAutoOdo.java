package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** a command that (ideally) drives the robot 2m forward. */
public class Drive5ftInAutoOdo extends CommandBase {
    private final Drivetrain drivetrain;
    private boolean finished = false;
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(0.6);
    private int schedulerIncrement = 0;

    /*
     * Creates a new Drive5ftInAutoOdo.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Drive5ftInAutoOdo(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.resetEncoders();
        drivetrain.zeroHeading();
        drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        finished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (schedulerIncrement % 10 == 0) {
            //System.out.println("X: "+drivetrain.getPose().getY());
            //System.out.println("Y: "+drivetrain.getPose().getX());
            System.out.println("R: "+drivetrain.getPose().getRotation().getDegrees());
            //System.out.println("T: "+drivetrain.getPose().getTranslation());
        } else {
            //System.out.println("I"+schedulerIncrement);
        }
        schedulerIncrement++;
        if (finished) {
            //System.out.println("Finished");
            drivetrain.drive(0, rateLimiter.calculate(0), 0, false);
        } else {
            //System.out.println("Not Finished");
            drivetrain.drive(0, rateLimiter.calculate(0.1), 0, false);
        }
        if (drivetrain.getPose().getY() >= 1) {
            //System.out.println("Finished (pose)");
            finished = true;
        }
        SmartDashboard.putNumber("BLEnc (rotations)", drivetrain.getRearLeftEncoder().getPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isStopped() && finished;
    }

    private boolean isStopped() {
        MecanumDriveWheelSpeeds speeds = drivetrain.getCurrentWheelSpeeds();
        return Math.abs(speeds.frontLeftMetersPerSecond) +
                Math.abs(speeds.frontRightMetersPerSecond) +
                Math.abs(speeds.rearLeftMetersPerSecond) +
                Math.abs(speeds.rearRightMetersPerSecond) > 0;
    }
}
