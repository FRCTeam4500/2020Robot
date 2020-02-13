/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IAngularVelocityGetterComponent;

/**
 * Add your docs here.
 */
public class AngularVelocityGetterDashboardDecorator extends DashboardDecorator
        implements IAngularVelocityGetterComponent {

    private IAngularVelocityGetterComponent angularVelocityGetterComponent;

    public AngularVelocityGetterDashboardDecorator(String name, String subsystem,
            IAngularVelocityGetterComponent angularVelocityGetterComponent) {
        super(name, subsystem);
        this.angularVelocityGetterComponent = angularVelocityGetterComponent;
        send();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kSpeedController.getWidgetName());
        builder.addDoubleProperty("Value", this::getAngularVelocity, null);
    }

    @Override
    public double getAngularVelocity() {
        return angularVelocityGetterComponent.getAngularVelocity();
    }
}
