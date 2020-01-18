/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.command;

import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.command.Swerve_BaseCommand;

/**
 * A command which drives the {@link Swerve} drive based on inputs from an {@link ISwerveOI},
 * relative to the field (in other words, invoking
 * {@link Swerve#moveFieldCentric(double, double, double)}).
 */
public class DriveCommand extends Swerve_BaseCommand {
    private ISwerveOI oi;

    /**
     * @param oi the {@link ISwerveOI} for this command to take inputs from
     * @see {@link Swerve_BaseCommand#Swerve_BaseCommand(Swerve)}.
     */
    public DriveCommand(Swerve swerve, ISwerveOI oi) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super(swerve);
        this.oi = oi;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        swerve.moveFieldCentric(oi.getX(), oi.getY(), oi.getZ());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean isInterrupted) {
    }


}
