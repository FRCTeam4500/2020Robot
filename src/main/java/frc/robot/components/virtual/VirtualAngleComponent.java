/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;

/**
 * A virtual component which implements {@link IAngleGetterComponent} and
 * {@link IAngleSetterComponent}.
 */
public class VirtualAngleComponent implements IAngleGetterComponent, IAngleSetterComponent {

    private double angle;

    @Override
    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public double getAngle() {
        return angle;
    }

}
