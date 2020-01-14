/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.components;

/**
 * An interface representing gyros. It is an extension of
 * {@link IAngleGetterComponent}, where {@link IGyroComponent#getAngle()}
 * usually returns the yaw of the gyro.
 */
public interface IGyroComponent extends IAngleGetterComponent {
    /**
     * Recalibrates the current angle of the gyro. In otherwords, the current angle
     * becomes zero, and future angles are computed based on this new zero.
     */
    void reset();
}
