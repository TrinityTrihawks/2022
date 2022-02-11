package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ResetGyro extends CommandBase {

    private Drivetrain drive;

    public ResetGyro(Drivetrain drivetrain) {
        drive = drivetrain;
        addRequirements(drivetrain);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.zeroHeading();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
