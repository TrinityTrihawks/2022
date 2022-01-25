package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.JoystickConstants;

public class DriveSingleJoystick extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier twistSupplier;
    private final DoubleSupplier throttleSupplier;

    public DriveSingleJoystick(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier twist, DoubleSupplier throttle) {
        this.drivetrain = drivetrain;
        this.xSupplier = x;
        this.ySupplier = y;
        this.twistSupplier = twist;
        this.throttleSupplier = throttle; // todo: remove unnecessary thises

        addRequirements(drivetrain);

    }
    // isaac helped 
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Drivetrain Initialized");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double twist = twistSupplier.getAsDouble();
        double throttle = throttleSupplier.getAsDouble();
        // scale x, y, and twist by throttle and sanity limit
        x = x * throttle * JoystickConstants.kStaticThrottleScalar;
        y = y * throttle * JoystickConstants.kStaticThrottleScalar * -1; //correct the y-axis (backwards is now backwards!)
        twist = twist * throttle * JoystickConstants.kStaticThrottleScalar;
        // Deadzone Logic
        x = x < JoystickConstants.kXDeadZone ? 0.0 : x;
        y = y < JoystickConstants.kYDeadZone ? 0.0 : y;
        twist = twist < JoystickConstants.kTwistDeadZone ? 0.0 : twist;
        
        System.out.print("X: "+x+"; ");
        System.out.print("Y: "+y+"; ");
        System.out.print("Twistation: "+twist+"; ");
        System.out.print("Throttle:"+throttle+"; ");
        
        System.out.println();

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
