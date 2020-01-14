/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.components.IDoubleSolenoidComponent;

/**
 * An {@link IDoubleSolenoidComponent} wrapper for {@link DoubleSolenoid}.
 */
public class DoubleSolenoidComponent extends DoubleSolenoid implements IDoubleSolenoidComponent {

    /**
     * @see DoubleSolenoid#DoubleSolenoid(int, int, int)
     */
    public DoubleSolenoidComponent(int moduleNumber, int forwardChannel, int reverseChannel) {
        super(moduleNumber, forwardChannel, reverseChannel);
    }

    /**
     * @see DoubleSolenoid#DoubleSolenoid(int, int)
     */
    public DoubleSolenoidComponent(int forwardChannel, int reverseChannel) {
        super(forwardChannel, reverseChannel);
    }

    @Override
    public boolean isExtended() {
        if (get() == Value.kForward) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void extend() {
        set(Value.kForward);
    }

    @Override
    public void retract() {
        set(Value.kReverse);
    }

}
