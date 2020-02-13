/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IAngleGetterComponent;

/**
 * A {@link DashboardDecorator} for any {@link IAngleGetterComponent}.
 */
public class AngleGetterDashboardDecorator extends DashboardDecorator implements IAngleGetterComponent {
    private IAngleGetterComponent getter;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param getter the {@link IAngleGetterComponent} component to decorate.
     */
    public AngleGetterDashboardDecorator(String name, String subsystem, IAngleGetterComponent getter) {
        super(name, subsystem);
        this.getter = getter;
        send();
    }

    @Override
    public double getAngle() {
        return getter.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Value", () -> -Math.toDegrees(getAngle()), null);
        builder.setSmartDashboardType(BuiltInWidgets.kGyro.getWidgetName());
    }
}
