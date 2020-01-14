/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import edu.wpi.first.wpilibj.PIDController;
import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.utility.AbstractPidControllerFactory;
import frc.robot.utility.PidCollector;

/**
 * A virtual angle component which is controlled by a {@link PIDController} and
 * implements {@link IAngleGetterComponent} and {@link IAngleSetterComponent}.
 * This is useful for simulating the fact that most angle setters tend to lag
 * behind their setpoint and do not reach their target instantaneously.
 */
public class VirtualPIDAngleComponent implements IAngleGetterComponent, IAngleSetterComponent {
    private PIDController controller;
    private double angle;

    private static double min = 0;
    private static double max = 10;

    public VirtualPIDAngleComponent(AbstractPidControllerFactory factory, String name) {

        controller = factory.makePidController(name, new PidCollector(this::getAngle), this::addToAngle);
        controller.enable();
    }

    /**
     * @return the {@link PIDController} which controls this virtual angle component
     */
    public PIDController getController() {
        return controller;
    }

    @Override
    public void setAngle(double angle) {
        controller.setSetpoint(angle);
    }

    @Override
    public double getAngle() {
        return angle;
    }

    /**
     * This clamps the given value and adds it to the angle.
     * 
     * @param value the value to add
     */
    private void addToAngle(double value) {
        if (value > max) {
            value = max;
        } else if (-min < value && value < min) {
            value = 0;
        } else if (value < -max) {
            value = -max;
        }
        angle += value;
    }
}
