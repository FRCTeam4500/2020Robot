package frc.robot.components;

/**
 * An interface for any component which controls output. This usually refers to
 * motors.
 */
public interface IOutputSetterComponent {
    /**
     * Sets the output of the component in relative units. For example, +1.0 is the
     * maximum forward output of the component, and -1.0 is the maximum reverse output
     * of the component.
     * 
     * @param output the desired speed of the component
     */
    public void setOutput(double output);
}
