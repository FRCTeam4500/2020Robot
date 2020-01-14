/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.IDoubleSolenoidComponent;

/**
 * A virtual {@link IDoubleSolenoidComponent} component.
 */
public class VirtualDoubleSolenoidComponent implements IDoubleSolenoidComponent {

    private boolean _isExtended;

    @Override
    public boolean isExtended() {
        return _isExtended;
    }

    @Override
    public void extend() {
        _isExtended = true;
    }

    @Override
    public void retract() {
        _isExtended = false;
    }

}
