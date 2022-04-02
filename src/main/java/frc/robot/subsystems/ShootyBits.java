package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BeamState;
import frc.robot.Constants.ShootyBitsConstants;

public class ShootyBits extends SubsystemBase implements IntakeBits, ShooterBits {
    private static ShootyBits subsystemInst = null;

    private final TalonSRX middleMotor = new TalonSRX(ShootyBitsConstants.kMiddleMotorPort);
    private final VictorSPX intakeMotor = new VictorSPX(ShootyBitsConstants.kIntakeMotorPort);
    private final VictorSPX shooterMotor = new VictorSPX(ShootyBitsConstants.kShooterMotorPort);

    private final VictorSPX armMotor = new VictorSPX(ShootyBitsConstants.kArmMotorPort);
    private final DigitalInput armLimit = new DigitalInput(ShootyBitsConstants.kLimitPort);

    private final DigitalInput midBeamSensor = new DigitalInput(ShootyBitsConstants.kLowBeamPort);
    private final DigitalInput highBeamSensor = new DigitalInput(ShootyBitsConstants.kHighBeamPort);

    private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

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

        armMotor.configFactoryDefault();

    }

    @Override
    public void setIntakeVoltage(double percentOutput) {
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setMiddleVoltage(double percentOutput) {
        middleMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    @Override
    public void setShooterVoltage(double percentOutput) {
        shooterMotor.set(ControlMode.PercentOutput, percentOutput * (Constants.kOptimalVoltage / getTotalVoltage()));
    }
    public boolean getArmLimit() {
        return armLimit.get();
    }
    @Override
    public void setArmVoltage(double percentOutput) {
        armMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public double getTotalVoltage() {
        return pdp.getVoltage();
    }

    @Override
    public BeamState getLowBeamState() {
        return BeamState.fromBoolean(midBeamSensor.get());
    }

    @Override
    public BeamState getHighBeamState() {
        return BeamState.fromBoolean(highBeamSensor.get());
    }

    @Override
    public Subsystem getAsSubsystem() {
        return this;
    }

    @Override
    public Constants.Color getDetectedColor() {

        return null;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lower ball \"slot\"", getLowBeamState().state);
        SmartDashboard.putBoolean("Upper ball \"slot\"", getHighBeamState().state);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}