/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * An abstract factory for creating {@link PIDController}s.
 */
public abstract class AbstractPIDControllerFactory {
    /**
     * Create a {@link PIDController}.
     * 
     * @param name   the name of this controller as it shows up in the dashboard
     * @param source the source which the controller pulls from
     * @param output the output which the controller writes to
     * @return
     */
    public abstract PIDController makePIDController(String name);
}
