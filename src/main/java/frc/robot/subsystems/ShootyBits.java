package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static Intake subsystemInst = null;

    private final TalonSRX intakeMotor = new TalonSRX(IntakeConstants.kIntakeMotorPort);
    private final DigitalInput limitSwitch = new DigitalInput(IntakeConstants.kLimitSwitchPort);

    /** Creates a new Intake. */
    /**
     * Use this method to create a Intake instance. This method ensures that the
     * Intake class is a singleton, aka, that only one Intake object ever
     * gets created
     */
    public static Intake getInstance() {
        if (subsystemInst == null) {
            subsystemInst = new Intake();
        } 
        return subsystemInst;
    }

    private Intake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setIntakeVoltage(double percentOutput) {
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public boolean getLimitSwitchState() {
        return limitSwitch.get();
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