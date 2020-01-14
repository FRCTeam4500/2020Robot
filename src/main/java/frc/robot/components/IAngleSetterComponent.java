package frc.robot.components;

/**
 * An interface for anything that has an angle which can be set to a desired
 * value. This is the interface used for PID positional motors, such as a
 * TalonSRX.
 */
public interface IAngleSetterComponent {
    /**
     * Sets the component to the desired angle. This angle should be measured in
     * radians using standard angles (east is zero, positive is counter clockwise,
     * negative is clockwise.)
     * 
     * @param angle the desired angle
     */
    public void setAngle(double angle);
}
