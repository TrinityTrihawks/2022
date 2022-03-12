package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootyBitsConstants;

public class ShootyBits extends SubsystemBase implements IntakeBits, ShooterBits {
    private static ShootyBits subsystemInst = null;

    private final TalonSRX middleMotor = new TalonSRX(ShootyBitsConstants.kMiddleMotorPort);
    private final VictorSPX intakeMotor = new VictorSPX(ShootyBitsConstants.kIntakeMotorPort);
    private final VictorSPX shooterMotor = new VictorSPX(ShootyBitsConstants.kShooterMotorPort);

    private final DigitalInput midBeamSensor = new DigitalInput(ShootyBitsConstants.kMidBeamPort);
    private final DigitalInput highBeamSensor = new DigitalInput(ShootyBitsConstants.kHighBeamPort);

    /**
     * Use this method to create a ShootyBits instance. This method ensures that the
     * ShootyBits class is a singleton, aka, that only one ShootyBits object ever
     * gets created
     */
    public static ShootyBits getInstance() {
        if (subsystemInst == null) {
            subsystemInst = new ShootyBits();
        }
        return subsystemInst;
    }

    private ShootyBits() {
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        middleMotor.configFactoryDefault();
        middleMotor.setNeutralMode(NeutralMode.Brake);

        shooterMotor.configFactoryDefault();
        shooterMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setIntakeVoltage(double percentOutput) {
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setMiddleVoltage(double percentOutput) {
        middleMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setShooterVoltage(double percentOutput) {
        shooterMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public boolean getMidBeamState() {
        return midBeamSensor.get();
    }

    public boolean getHighBeamState() {
        return highBeamSensor.get();
    }

    public Subsystem getAsSubsystem() {
        return this;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lower ball \"slot\"", getMidBeamState());
        SmartDashboard.putBoolean("Upper ball \"slot\"", getHighBeamState());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}