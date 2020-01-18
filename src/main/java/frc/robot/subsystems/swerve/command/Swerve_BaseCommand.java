/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.command;

import frc.robot.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The base command for all {@link Swerve} commands.
 */
public abstract class Swerve_BaseCommand extends CommandBase {
    protected Swerve swerve;

    /**
     * Creates a command using the specified {@link Swerve} drive.
     * 
     * @param swerve the swerve drive this command should use
     */
    public Swerve_BaseCommand(Swerve swerve) {
        addRequirements(swerve);
        this.swerve = swerve;
    }
}
