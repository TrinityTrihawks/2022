package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntakeUntilLimitSwitch extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Intake intake;

    /**
     * Creates a new RunIntakeUntilLimitSwitch.
     *
     * @param intake The subsystem used by this command.
     */
    public RunIntakeUntilLimitSwitch(Intake subsystem) {
        intake = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeVoltage(0.5);
    }

    @Override
    public void execute() {
        if (intake.getLimitSwitchState()) {
            intake.setIntakeVoltage(0.0);
        }
        //System.out.println(intake.getLimitSwitchState());
    }

    @Override
    public boolean isFinished() {
        return false;
        // return intake.getLimitSwitchState();
    }
}
