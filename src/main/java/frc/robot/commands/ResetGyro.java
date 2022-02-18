package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetGyro extends CommandBase {

    private Drivetrain drive;
    private Timer timer = new Timer();
    private double waitTime = 0;

    public ResetGyro(Drivetrain drivetrain, double waitTime) {
        drive = drivetrain;
        this.waitTime = waitTime;
        addRequirements(drivetrain);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.zeroHeading();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(0, 0, 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(waitTime);
    }
}
