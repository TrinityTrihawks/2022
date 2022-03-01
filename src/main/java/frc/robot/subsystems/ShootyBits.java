package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class ShootyBits extends SubsystemBase {
    private static ShootyBits subsystemInst = null;

    private final TalonSRX lowMotor = new TalonSRX(IntakeConstants.kLowMotorPort);
    private final TalonSRX midMotor = new TalonSRX(IntakeConstants.kMiddleMotorPort);
    private final TalonSRX highMotor = new TalonSRX(IntakeConstants.kHighMotorPort);

    private final DigitalInput lowBeamSensor = new DigitalInput(0);
    private final DigitalInput midBeamSensor = new DigitalInput(0);
    private final DigitalInput highBeamSensor = new DigitalInput(0);

    /** Creates a new ShootyBits. */
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
        lowMotor.configFactoryDefault();
        lowMotor.setNeutralMode(NeutralMode.Brake);

        midMotor.configFactoryDefault();
        midMotor.setNeutralMode(NeutralMode.Brake);

        highMotor.configFactoryDefault();
        highMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setLowVoltage(double percentOutput) {
        lowMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setMidVoltage(double percentOutput) {
        midMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void setHighVoltage(double percentOutput) {
        highMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public boolean getLowBeamState() {
        return lowBeamSensor.get();
    }

    public boolean getMidBeamState() {
        return midBeamSensor.get();
    }

    public boolean getHighBeamState() {
        return highBeamSensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}