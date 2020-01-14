/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/**
 * An interface for double solenoids. This usually refers to
 * {@link edu.wpi.first.wpilibj.DoubleSolenoid DoubleSolenoid}, but can also
 * refer to simulated components such as
 * {@link frc.robot.components.virtual.VirtualDoubleSolenoidComponent
 * VirtualDoubleSolenoidComponent}.
 */
public interface IDoubleSolenoidComponent {
    /**
     * Returns whether the solenoid is extended or not.
     * 
     * @return is the solenoid extended
     */
    boolean isExtended();

    /**
     * Extends the solenoid, if not already extended.
     */
    void extend();

    /**
     * Retract the solenoid, if not already retracted.
     */
    void retract();

    /**
     * Toggles the state of the solenoid. If it is extended, retract the solenoid,
     * and vice versa.
     */
    default void toggleExtension() {
        if (isExtended()) {
            retract();
        } else {
            extend();
        }
    }
}
