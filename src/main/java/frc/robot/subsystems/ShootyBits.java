package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.BeamState;
import static frc.robot.Constants.ShootyBitsConstants;
import frc.robot.Constants;

public class ShootyBits extends SubsystemBase implements IntakeBits, ShooterBits {
    private static ShootyBits subsystemInst = null;

    private final TalonSRX middleMotor = new TalonSRX(ShootyBitsConstants.kMiddleMotorPort);
    private final VictorSPX intakeMotor = new VictorSPX(ShootyBitsConstants.kIntakeMotorPort);
    private final VictorSPX shooterMotor = new VictorSPX(ShootyBitsConstants.kShooterMotorPort);

    private final TalonSRX armMotor = new TalonSRX(ShootyBitsConstants.kArmMotorPort);
    private final DigitalInput limitSwitchUpper = new DigitalInput(ShootyBitsConstants.kLimitPort);

    private final DigitalInput midBeamSensor = new DigitalInput(ShootyBitsConstants.kLowBeamPort);
    private final DigitalInput highBeamSensor = new DigitalInput(ShootyBitsConstants.kHighBeamPort);

    private final ColorSensorV3 detector = new ColorSensorV3(I2C.Port.kOnboard);
    private final ColorMatch matchProcessor = new ColorMatch();

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


        matchProcessor.addColorMatch(Constants.Color.toWpiColor(Constants.Color.RED));
        matchProcessor.addColorMatch(Constants.Color.toWpiColor(Constants.Color.BLUE));
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
        shooterMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    
    @Override
    public void setArmVoltage(double percentOutput) {
        if (!limitSwitchUpper.get()) {// TODO add other limit
            armMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            armMotor.set(ControlMode.PercentOutput, 0);
        }
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
        Color colorDetected = detector.getColor();
        ColorMatchResult matchedColor = matchProcessor.matchClosestColor(colorDetected);
    
        if (matchedColor.color == Constants.Color.toWpiColor(Constants.Color.RED)) {
            return Constants.Color.RED;
        } else if (matchedColor.color == Constants.Color.toWpiColor(Constants.Color.BLUE)){
            return Constants.Color.BLUE;
        } else {
            return Constants.Color.NONE;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Lower ball \"slot\"", getLowBeamState().state);
        SmartDashboard.putBoolean("Upper ball \"slot\"", getHighBeamState().state);

        Color colorDetected = detector.getColor();
        SmartDashboard.putNumber("detected red", colorDetected.red);
        SmartDashboard.putNumber("detected green", colorDetected.green);
        SmartDashboard.putNumber("detected blue", colorDetected.blue);

        /*
        ColorMatchResult matchedColor = matchProcessor.matchClosestColor(colorDetected);
        if (matchedColor.color == Constants.Color.toWpiColor(Constants.Color.RED)) {
            SmartDashboard.putString("detected color", "RED");
        } else if (matchedColor.color == Constants.Color.toWpiColor(Constants.Color.BLUE)){
            SmartDashboard.putString("detected color", "BLUE");
        } else {
            SmartDashBoard.putString("detected color", "NONE");
        }
        //*/
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}