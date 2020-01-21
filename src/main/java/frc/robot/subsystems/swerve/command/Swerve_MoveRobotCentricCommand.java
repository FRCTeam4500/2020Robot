/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.command;

import frc.robot.subsystems.swerve.Swerve;

/**
 * Add your docs here.
 */
public class Swerve_MoveRobotCentricCommand extends Swerve_BaseCommand {
    /**
     * Add your docs here.
     */
    private double x, y, w;

    public Swerve_MoveRobotCentricCommand(Swerve swerve, double x, double y, double w) {
        super(swerve);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.x = x;
        this.y = y;
        this.w = w;
    }

    // Called once when the command executes
    @Override
    public void initialize() {
        swerve.moveRobotCentric(x, y, w);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
