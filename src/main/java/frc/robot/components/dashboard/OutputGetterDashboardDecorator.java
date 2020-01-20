/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IOutputGetterComponent;

/**
 * Add your docs here.
 */
public class OutputGetterDashboardDecorator extends DashboardDecorator implements IOutputGetterComponent{

    private IOutputGetterComponent speedGetterComponent;
    public OutputGetterDashboardDecorator(IOutputGetterComponent speedGetterComponent, String name, String subsystem) {
        super(name, subsystem);
        this.speedGetterComponent = speedGetterComponent;
        send();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kSpeedController.getWidgetName());
        builder.addDoubleProperty("Value", this::getOutput, null);
    }

    @Override
    public double getOutput() {
        return speedGetterComponent.getOutput();
    }
}
