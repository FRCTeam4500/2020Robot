package frc.robot.components;

/**
 * An interface for any component which controls speed. This usually refers to
 * motors.
 */
public interface ISpeedSetterComponent {
    /**
     * Sets the speed of the component in relative units. For example, +1.0 is the
     * maximum forward speed of the component, and -1.0 is the maximum reverse speed
     * of the component.
     * 
     * @param speed the desired speed of the component
     */
    public void setSpeed(double speed);
}
