/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IOutputSetterComponent;

/**
 * A {@link DashboardDecorator} for any {@link IOutputSetterComponent} component.
 */
public class OutputSetterDashboardDecorator extends DashboardDecorator implements IOutputSetterComponent {

    private double lastSetSpeed;
    private IOutputSetterComponent setter;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param setter the {@link IOutputSetterComponent} component to decorate
     */
    public OutputSetterDashboardDecorator(String name, String subsystem, IOutputSetterComponent setter) {
        super(name, subsystem);
        this.setter = setter;
        send();
    }

    @Override
    public void setOutput(double speed) {
        lastSetSpeed = speed;
        setter.setOutput(speed);
    }

    /**
     * Gets the last set speed set by {@link #setOutput(double)}. The speed sent is
     * not always the final speed of the component, since the component may
     * transform the input into another value.
     * 
     * @return the last set speed
     */
    public double getLastSetSpeed() {
        return lastSetSpeed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kSpeedController.getWidgetName());
        builder.addDoubleProperty("Value", this::getLastSetSpeed, null);
    }
}
