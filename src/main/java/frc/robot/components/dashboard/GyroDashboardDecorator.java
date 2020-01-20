/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.IGyroComponent;

/**
 * A {@link #DashboardDecorator} for any {@link #IGyroComponent} component.
 */
public class GyroDashboardDecorator extends DashboardDecorator implements IGyroComponent {

    private IGyroComponent gyro;

    /**
     * See {@link DashboardDecorator#DashboardDecorator(String, String)} for more
     * details.
     * 
     * @param gyro the {@link IGyroComponent} component to decorate
     */
    public GyroDashboardDecorator(String name, String subsystem, IGyroComponent gyro) {
        super(name, subsystem);
        this.gyro = gyro;
        send();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public double getAngle() {
        return gyro.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(BuiltInWidgets.kGyro.getWidgetName());
        builder.addDoubleProperty("Value", this::getAngle, null);

    }
}
