
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDoubleJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier x1Supplier;
    private final DoubleSupplier x2Supplier;
    private final DoubleSupplier y1Supplier;
    private final DoubleSupplier y2Supplier;
    private final DoubleSupplier throttleSupplier;

    public DriveDoubleJoystick(Drivetrain drivetrain, DoubleSupplier x1, DoubleSupplier x2, DoubleSupplier y1, DoubleSupplier y2, DoubleSupplier throttle) {
        this.drivetrain = drivetrain;
        x1Supplier = x1;
        x2Supplier = x2;
        y1Supplier = y1;
        y2Supplier = y2;
        throttleSupplier = throttle; 
        addRequirements(drivetrain);

    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x1 = x1Supplier.getAsDouble();
        double x2 = x2Supplier.getAsDouble();
        double y1 = y1Supplier.getAsDouble();
        double y2 = y2Supplier.getAsDouble();
        double throttle = throttleSupplier.getAsDouble();
        
        // Deadzone Logic
        x1 = Math.abs(x1) < JoystickConstants.kXDeadZone ? 0.0 : x1;
        x2 = Math.abs(x2) < JoystickConstants.kXDeadZone ? 0.0 : x2;
        y1 = Math.abs(y1) < JoystickConstants.kYDeadZone ? 0.0 : y1;
        y2 = Math.abs(y2) < JoystickConstants.kYDeadZone ? 0.0 : y2;

        // Double joystick math (courtesy of Peter, Veronica, Luke & Michael)
        double y = (y1 + y2) / 2;
        double twist = (y2 - y1) / 2;

        double x;
        if (x1 * x2 < 0) {
            x = 0;
        } else {
            x = Math.abs (x2) < Math.abs (x1) ? x2 : x1;
        }

        
        throttle = (-throttle + 1)/2;

        // scale x, y, and twist by throttle and sanity limit
        x = x * throttle * JoystickConstants.kStaticThrottleScalar;
        y = y * throttle * JoystickConstants.kStaticThrottleScalar * -1; //correct the y-axis (backwards is now backwards!)
        twist = twist * throttle * JoystickConstants.kStaticThrottleScalar;

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Twistation", twist);
        SmartDashboard.putNumber("Throttle", throttle);

        drivetrain.drive(y, x, twist, false);
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
