package frc.vitruvianlib.utils;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickWrapper extends Joystick {

    private boolean[] invertedAxis;
    public JoystickWrapper(int port) {
        super(port);
        invertedAxis = new boolean[10];
    }

    public void invertRawAxis(int axis, boolean inverted) {
        invertedAxis[axis] = inverted;
    }

    @Override
    public double getRawAxis(int axis) {
        return invertedAxis[axis] ? -super.getRawAxis(axis) : super.getRawAxis(axis);
    }
}
