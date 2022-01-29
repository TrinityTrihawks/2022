package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive5ftInAutoOdo extends CommandBase {
    private final Drivetrain drivetrain;
    private boolean finished = false;

    /**
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
        drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (finished) {
            System.out.println("Finished");
            drivetrain.drive(0, 0, 0, false);
        } else {
            System.out.println("Not Finished");
            drivetrain.drive(0, 0.3, 0, false);
        }
        if (drivetrain.getPose().getY() >= 2) {
            System.out.println("Finished (pose)");
            finished = true;
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
        MecanumDriveWheelSpeeds speeds = drivetrain.getCurrentWheelSpeeds();
        return speeds.frontLeftMetersPerSecond +
               speeds.frontRightMetersPerSecond +
               speeds.rearLeftMetersPerSecond +
               speeds.rearRightMetersPerSecond > 0;
    }
}
