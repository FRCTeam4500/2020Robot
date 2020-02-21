/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components.hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;

import frc.robot.components.IOutputSetterComponent;
import frc.robot.components.ISmartMotorComponent;

/**
 * An {@link IOutputSetterComponent}, {@link IAngleSetterComponent}, and
 * {@link IAngleGetterComponent} wrapper for {@link TalonSRX}.
 */
public class TalonSRXComponent extends TalonSRX
        implements ISmartMotorComponent {

    public static final double TICKS_PER_DEGREE = 4096 / 360.0;
    public static final double TICKS_PER_RADIAN = 4096 / Math.PI / 2.0;

    /**
     * @see TalonSRX#TalonSRX(int)
     */
    public TalonSRXComponent(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setOutput(double speed) {
        set(ControlMode.PercentOutput, -speed);
    }

    @Override
    public void setAngle(double angle) {
        set(ControlMode.MotionMagic, -angle * TICKS_PER_RADIAN);
    }

    @Override
    public double getAngle() {
        return -getSelectedSensorPosition() / TICKS_PER_RADIAN;
    }

    @Override
    public double getAngularVelocity() {
        return -getSelectedSensorVelocity() / TICKS_PER_RADIAN * 10;
    }

    @Override
    public double getOutput() {
        return -getMotorOutputPercent();
    }

    @Override
    public void setAngularVelocity(double velocity) {
        set(ControlMode.Velocity, -velocity * TICKS_PER_RADIAN / 10.0);
    }
}
