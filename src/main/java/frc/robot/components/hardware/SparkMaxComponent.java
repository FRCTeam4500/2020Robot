/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.robot.components.ISmartMotorComponent;

/**
 * Add your docs here.
 */
public class SparkMaxComponent extends CANSparkMax implements ISmartMotorComponent{

    public SparkMaxComponent(int deviceID, MotorType type) {
        super(deviceID, type);
    }

    @Override
    public double getAngle() {
        return -getEncoder().getPosition()*Math.PI * 2;
    }

    @Override
    public void setAngle(double angle) {
        getPIDController().setReference(-angle/2/Math.PI, ControlType.kPosition);
        
    }

    @Override
    public void setOutput(double output) {
        set(-output);
    }

    @Override
    public double getOutput() {
        return -get();
    }

    @Override
    public double getAngularVelocity() {
        return -getEncoder().getVelocity()/60.0*Math.PI * 2;
    }

    @Override
    public void setAngularVelocity(double velocity) {
        getPIDController().setReference(-velocity*60.0/Math.PI/2.0, ControlType.kVelocity);
    }

}
