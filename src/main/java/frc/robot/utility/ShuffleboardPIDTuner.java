/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * A decorator that enables tuning for instances of {@link PIDController} in Shuffleboard. Must be
 * in test mode to tune.
 */
public class ShuffleboardPIDTuner implements Sendable {
    private PIDController controller;
    private double setpointDelta = 0;

    public ShuffleboardPIDTuner(PIDController controller, String subsystem, String name) {
        SendableRegistry.setName(controller, name);
        this.controller = controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("__kP", controller::getP, controller::setP);
        builder.addDoubleProperty("_kI", controller::getI, controller::setI);
        builder.addDoubleProperty("kD", controller::getD, controller::setD);
        builder.addDoubleProperty("setpointDelta", () -> setpointDelta, value -> {
            setpointDelta = value;
            controller.setTolerance(value);
        });
        builder.addDoubleProperty("error", controller::getPositionError, null);
    }

}
