/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IAngularVelocitySetterComponent;

/**
 * Add your docs here.
 */
public class AngularVelocitySetterDashboardDecorator extends DashboardDecorator
        implements IAngularVelocitySetterComponent {
    private IAngularVelocitySetterComponent angularVelocitySetterComponent;
    private double lastSetAngularVelocity = 0.0;

    public AngularVelocitySetterDashboardDecorator(String name, String subsystem, IAngularVelocitySetterComponent angularVelocitySetterComponent) {
        super(name, subsystem);
        this.angularVelocitySetterComponent = angularVelocitySetterComponent;
        send();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kSpeedController.getWidgetName());
        builder.addDoubleProperty("Value", this::getLastSetAngularVelocity, null);
    }

    /**
     * @return the lastSetAngularVelocity
     */
    public double getLastSetAngularVelocity() {
        return lastSetAngularVelocity;
    }

    @Override
    public void setAngularVelocity(double velocity) {
        lastSetAngularVelocity = velocity;
        angularVelocitySetterComponent.setAngularVelocity(velocity);

    }
}
