package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestSlewLimiter extends CommandBase {
    private int input = 1;
    private int t = 1;
    private double output = 1;
    private SlewRateLimiter filter = new SlewRateLimiter(4, 1);

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (t % 50 == 0) {
            input = -input;
        }
        output = filter.calculate(input);
        System.out.println(input + "," + output);
        t++;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
