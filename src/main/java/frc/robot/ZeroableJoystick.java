package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ZeroableJoystick extends Joystick {
    private double offsetX = 0;
    private double offsetY = 0;
    private double offsetTwist = 0;
    private double offsetThrottle = 0;

    public ZeroableJoystick(int port) {
        super(port);
    }

    public void zero() {
        offsetX = getX();
        offsetY = getY();
        offsetTwist = getTwist();
        offsetThrottle = getThrottle();
        System.out.println("00000000000000000 ZEROED 00000000000000000");
    }

    public double getZeroedX() {
        return getX() - offsetX;
    }

    public double getZeroedY() {
        return getY() - offsetY;
    }

    public double getZeroedTwist() {
        return getTwist() - offsetTwist;
    }

    public double getZeroedThrottle() {
        return getThrottle() - offsetThrottle;
    }
}