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
import frc.robot.components.ISpeedSetterComponent;

/**
 * An {@link ISpeedSetterComponent}, {@link IAngleSetterComponent}, and
 * {@link IAngleGetterComponent} wrapper for {@link TalonSRX}.
 */
public class TalonSRXComponent extends TalonSRX
        implements ISpeedSetterComponent, IAngleSetterComponent, IAngleGetterComponent {

    public static final double TICKS_PER_DEGREE = 16.2539;
    public static final double TICKS_PER_RADIAN = TICKS_PER_DEGREE * 360 / 2 / Math.PI;

    /**
     * @see TalonSRX#TalonSRX(int)
     */
    public TalonSRXComponent(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void setSpeed(double speed) {
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
}
