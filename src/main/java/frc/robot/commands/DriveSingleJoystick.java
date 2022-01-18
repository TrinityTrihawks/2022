package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveSingleJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;

    public DriveSingleJoystick(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        this.drivetrain = drivetrain;
        this.xSupplier = x;
        this.ySupplier = y;
        this.rotSupplier = rot;
        addRequirements(drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Drivetrain Initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        x = Math.pow(x, 2) * Math.signum(x);

        double y = ySupplier.getAsDouble();
        double rot = rotSupplier.getAsDouble();

        System.out.print("X: "+x+"; ");
        System.out.print("Y: "+y+"; ");
        System.out.print("Rotation: "+rot+"; ");
        
        System.out.println();

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Rotation", rot);

        drivetrain.drive(x, y, rot, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setDriveMotorControllersVolts(new MecanumDriveMotorVoltages(0, 0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
