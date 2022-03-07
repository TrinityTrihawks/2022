package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ZeroableJoystick extends Joystick {
    private double offsetX = 0;
    private double offsetY = 0;
    private double offsetTwist = 0;
    private double offsetThrottle = 0;
    private String name;

    public ZeroableJoystick(int port, String name) {
        super(port);
        this.name = name;
    }

    public void zero() {
        offsetX = getX();
        offsetY = getY();
        offsetTwist = getTwist();
        offsetThrottle = getThrottle();
        System.out.println("00000000000000000 " + name + " ZEROED 00000000000000000");
    }

    public double getZeroedX() {
        double retval = getX() - offsetX;
        return retval > 1? 1 : (retval < -1? -1 : retval);
    }

    public double getZeroedY() {
        double retval = getY() - offsetY;
        return retval > 1? 1 : (retval < -1? -1 : retval);
    }

    public double getZeroedTwist() {
        double retval = getTwist() - offsetTwist;
        return retval > 1? 1 : (retval < -1? -1 : retval);
    }

    public double getZeroedThrottle() {
        double retval = getThrottle() - offsetThrottle;
        return retval > 1? 1 : (retval < -1? -1 : retval);
    }
}