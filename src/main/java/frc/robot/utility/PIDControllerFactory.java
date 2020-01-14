/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * The default {@link PIDController} factory, which creates a controller with all constants set to
 * zero.
 */
public class PIDControllerFactory extends AbstractPIDControllerFactory {

    @Override
    public PIDController makePIDController(String name) {
        var controller = new PIDController(0, 0, 0);
        SendableRegistry.setName(controller, name);
        return controller;
    }
}


