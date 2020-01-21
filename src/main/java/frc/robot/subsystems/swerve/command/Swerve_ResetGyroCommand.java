/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.command;

import frc.robot.subsystems.swerve.Swerve;

/**
 * A command wrapper for {@link Swerve#resetGyro()}.
 */
public class Swerve_ResetGyroCommand extends Swerve_BaseCommand {

    /**
     * @see {@link Swerve_BaseCommand#Swerve_BaseCommand(Swerve)}
     */
    public Swerve_ResetGyroCommand(Swerve swerve) {
        super(swerve);
    }

    @Override
    public void initialize() {
        swerve.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
