/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

/**
 * An interface which defines all the methods required in order to drive a {@link Swerve} drive.
 */
public interface ISwerveOI {
    /**
     * Gets the X input at which to drive the {@link Swerve} drive.
     * 
     * @return the X input
     */
    public double getX();

    /**
     * Gets the Y input at which to drive the {@link Swerve} drive.
     * 
     * @return the Y input
     */
    public double getY();

    /**
     * Gets the Z input at which to drive the {@link Swerve} drive.
     * 
     * @return the Z input
     */
    public double getZ();
}
