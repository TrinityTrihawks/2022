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
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier y1Supplier;
    private final DoubleSupplier y2Supplier;
    private final DoubleSupplier throttleSupplier;

    public DriveDoubleJoystick(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y1, DoubleSupplier y2, DoubleSupplier throttle) {
        this.drivetrain = drivetrain;
        xSupplier = x;
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
        double x = xSupplier.getAsDouble();
        double y1 = y1Supplier.getAsDouble();
        double y2 = y2Supplier.getAsDouble();
        double throttle = throttleSupplier.getAsDouble();
        
        // Deadzone Logic
        x = Math.abs(x) < JoystickConstants.kXDeadZone ? 0.0 : x;
        y1 = Math.abs(y1) < JoystickConstants.kYDeadZone ? 0.0 : y1;
        y2 = Math.abs(y2) < JoystickConstants.kYDeadZone ? 0.0 : y2;

        // Double joystick math (courtesy of Peter & Veronica)
        double y = (y1 + y2) / 2;
        double twist = (y2 - y1) / 2;

        throttle = (-throttle + 1)/2;

        // scale x, y, and twist by throttle and sanity limit
        x = x * throttle * JoystickConstants.kStaticThrottleScalar;
        y = y * throttle * JoystickConstants.kStaticThrottleScalar * -1; //correct the y-axis (backwards is now backwards!)
        twist = twist * throttle * JoystickConstants.kStaticThrottleScalar;

        // System.out.print("X: "+x+"; ");
        // System.out.print("Y: "+y+"; ");
        // System.out.print("Twistation: "+twist+"; ");
        // System.out.print("Throttle:"+throttle+"; ");
        
        // System.out.println();

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
