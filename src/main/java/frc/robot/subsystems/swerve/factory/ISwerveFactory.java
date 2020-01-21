/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.factory;

import frc.robot.subsystems.swerve.Swerve;

/**
 * An interface for factories which create subsystem instances of {@link Swerve}.
 */
public interface ISwerveFactory {
    /**
     * Creates a configured instance of the {@link Swerve} subsystem.
     * 
     * @return the configured subsystem
     */
    Swerve makeSwerve();
}
