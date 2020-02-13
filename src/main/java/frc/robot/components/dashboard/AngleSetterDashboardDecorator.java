/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IAngleSetterComponent;

/**
 * A {@link DashboardDecorator} for any {@link IAngleSetterComponent} component.
 */
public class AngleSetterDashboardDecorator extends DashboardDecorator implements IAngleSetterComponent {

    private double lastSetAngle = 0;
    private IAngleSetterComponent setter;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param setter the {@link IAngleSetterComponent} to decorate
     */
    public AngleSetterDashboardDecorator(String name, String subsystem, IAngleSetterComponent setter) {
        super(name + " " + "Angle Setter Component", subsystem);
        this.setter = setter;
        send();
    }

    @Override
    public void setAngle(double angle) {
        this.lastSetAngle = angle;
        setter.setAngle(angle);
    }

    /**
     * Gets the last angle set to this component. Often, the sent angle is
     * transformed before being sent to the component, so the last set angle and the
     * actual angle are often different. This method gets the raw angle sent to the
     * component.
     * 
     * @return the raw, last set angle
     */
    public double getLastSetAngle() {
        return lastSetAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kGyro.getWidgetName());
        builder.addDoubleProperty("Value", () -> -Math.toDegrees(getLastSetAngle()), null);
    }
}
