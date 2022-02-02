package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static Intake subsystemInst = null;

    private final TalonSRX intakeMotor = new TalonSRX(IntakeConstants.kIntakeMotorPort);
    private final DigitalInput limitSwitch = new DigitalInput(0);

    /** Creates a new Intake. */
    // todo: remove hungarian pollution
    /**
     * Use this method to create a Intake instance. This method ensures that the
     * Intake class is a singleton, aka, that only one Intake object ever
     * gets created
     */
    public static Intake getInstance() {
        if (subsystemInst == null) {
            return new Intake();
        } else {
            return subsystemInst;
        }
    }

    private Intake() {
    }

    public void setIntakeVoltage(double percentOutput) {
        if (limitSwitch.get()) {
            return;
        }
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public boolean getLimitSwitchState() {
        return limitSwitch.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (limitSwitch.get()) {
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}