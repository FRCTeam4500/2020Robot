/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.mock;

import frc.robot.components.ISmartMotorComponent;

public class MockSmartMotorComponent implements ISmartMotorComponent {

    private double lastSetAngle, returnGetAngle, lastSetOutput, returnGetOutput, lastSetAngularVelocity,
            returnAngularVelocity;

    @Override
    public double getAngle() {
        return returnGetAngle;
    }

    public void setReturnGetAngle(double angle){
        returnGetAngle = angle;
    }

    public double getLastSetAngularVelocity() {
        return lastSetAngularVelocity;
    }

    public void setLastSetAngularVelocity(double lastSetAngularVelocity) {
        this.lastSetAngularVelocity = lastSetAngularVelocity;
    }

    public double getLastSetOutput() {
        return lastSetOutput;
    }


    public void setLastSetOutput(double lastSetOutput) {
        this.lastSetOutput = lastSetOutput;
    }

    public double getLastSetAngle() {
        return lastSetAngle;
    }

    public void setLastSetAngle(double lastSetAngle) {
        this.lastSetAngle = lastSetAngle;
    }

    @Override
    public void setAngle(double angle) {
        setLastSetAngle(angle);
    }

    @Override
    public void setOutput(double output) {
        setLastSetOutput(output);
    }

    @Override
    public double getOutput() {
        return returnGetOutput;
    }

    @Override
    public double getAngularVelocity() {
        return returnAngularVelocity;
    }

    @Override
    public void setAngularVelocity(double velocity) {
        setLastSetAngularVelocity(velocity);
    }
}
