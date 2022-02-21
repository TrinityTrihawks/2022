package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.JoystickConstants;

public class DriveDoubleJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xLeftSupplier;
    private final DoubleSupplier xRightSupplier;
    private final DoubleSupplier yLeftSupplier;
    private final DoubleSupplier yRightSupplier;
    private final DoubleSupplier throttleSupplier;

    
    public DriveDoubleJoystick(Drivetrain drivetrain, DoubleSupplier xLeft, DoubleSupplier xRight,
     DoubleSupplier yLeft, DoubleSupplier yRight, DoubleSupplier throttle) {
        this.drivetrain = drivetrain;
        xLeftSupplier = xLeft;
        xRightSupplier = xRight;
        yLeftSupplier = yLeft;
        yRightSupplier = yRight;
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
        double xLeft = xLeftSupplier.getAsDouble();
        double xRight = xRightSupplier.getAsDouble();
        double yLeft = yLeftSupplier.getAsDouble();
        double yRight = yRightSupplier.getAsDouble();
        double throttle = throttleSupplier.getAsDouble();
        
        // Deadzone Logic
        xLeft = Math.abs(xLeft) < JoystickConstants.kXDeadZone ? 0.0 : xLeft;
        xRight = Math.abs(xRight) < JoystickConstants.kXDeadZone ? 0.0 : xRight;
        yLeft = Math.abs(yLeft) < JoystickConstants.kYDeadZone ? 0.0 : yLeft;
        yRight = Math.abs(yRight) < JoystickConstants.kYDeadZone ? 0.0 : yRight;

        // Double joystick math (courtesy of Peter, Veronica, Luke & Michael)
        double y = (yLeft + yRight) / 2;
        double twist = (yLeft - yRight) / 2;

        double x;
        if (xLeft * xRight < 0) {
            x = 0;
        } else {
            x = Math.abs(xRight) < Math.abs(xLeft) ? xRight : xLeft;
        }

        
        throttle = (-throttle + 1)/2;

        // scale x, y, and twist by throttle and sanity limit
        x = x * throttle * JoystickConstants.kStaticThrottleScalar;
        y = y * throttle * JoystickConstants.kStaticThrottleScalar * -1; //correct the y-axis (backwards is now backwards!)
        twist = -twist * throttle * JoystickConstants.kStaticThrottleScalar;

        
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Twistation", twist);
        SmartDashboard.putNumber("Throttle", throttle);

        drivetrain.drive(x, y, twist, false);
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
