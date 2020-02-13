/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.IOutputSetterComponent;

/**
 * A virtual {@link IOutputSetterComponent} component.
 */
public class VirtualSpeedComponent implements IOutputSetterComponent {

    private double speed;

    /**
     * The current speed of this virtual component.
     * 
     * @return the current virtual speed
     */
    public double getSpeed() {
        return speed;
    }

    @Override
    public void setOutput(double speed) {
        this.speed = speed;
    }
}
