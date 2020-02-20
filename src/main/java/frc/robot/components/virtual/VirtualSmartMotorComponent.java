/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.virtual;

import frc.robot.components.ISmartMotorComponent;

/**
 * Add your docs here.
 */
@Deprecated
public class VirtualSmartMotorComponent implements ISmartMotorComponent{

    private double currentRadians = 0.0;
    private double radiansPerSecond = 0.0;
    private double radiansPerSecondPerFullOutput = 2 * Math.PI;

    @Override
    public double getAngle() {
        return currentRadians;
    }

    @Override
    public void setAngle(double angle) {
        currentRadians = angle;
    }

    @Override
    public void setOutput(double output) {

    }

    @Override
    public double getOutput() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getAngularVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setAngularVelocity(double velocity) {
        // TODO Auto-generated method stub

    }
}
