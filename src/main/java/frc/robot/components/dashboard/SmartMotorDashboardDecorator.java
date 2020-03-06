/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.components.ISmartMotorComponent;

/**
 * Add your docs here.
 */
public class SmartMotorDashboardDecorator extends DashboardDecorator implements ISmartMotorComponent {
    private ISmartMotorComponent component;

    public SmartMotorDashboardDecorator(String name, String subsystem, ISmartMotorComponent component) {
        super(name, subsystem);
        this.component = component;
    }
    @Override
    public void setOutput(double output) {
        component.setOutput(output);
    }
    @Override
    public void setAngle(double angle) {
        component.setAngle(angle);
    }
    @Override
    public double getOutput() {
        return component.getOutput();
    }
    @Override
    public double getAngularVelocity() {
        return component.getAngularVelocity();
    }
    @Override
    public void setAngularVelocity(double velocity) {
        component.setAngularVelocity(velocity);
    }
    @Override
    public double getAngle() {
        return component.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
    
}
