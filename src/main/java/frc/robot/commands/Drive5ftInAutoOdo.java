package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** a command that (ideally) drives the robot 2m forward. */
public class Drive5ftInAutoOdo extends CommandBase {
    private final Drivetrain drivetrain;
    private boolean finished = false;
    private Timer timer = new Timer();
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
        drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        finished = false;
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (finished) {
            drivetrain.drive(0, 0, 0, false);
        } else {
            drivetrain.drive(0.1, 0, 0, true);
        }
        if (drivetrain.getPose().getX() >= 1.524) { // meters
            finished = true;
        }
        if ((timer.get() * 2) % 1 == 0) {
            System.out.println(drivetrain.getPose().getX() + "," +
                               drivetrain.getPose().getY() + "," +
                               drivetrain.getPose().getRotation().getDegrees());
        }
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
                Math.abs(speeds.rearRightMetersPerSecond) == 0;
    }
}
