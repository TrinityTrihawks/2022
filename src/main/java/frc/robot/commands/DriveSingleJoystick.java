package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSingleJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier thetaSupplier;

    public DriveSingleJoystick(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        this.drivetrain = drivetrain;
        this.xSupplier = x;
        this.ySupplier = y;
        this.thetaSupplier = theta;
        addRequirements(drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
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
