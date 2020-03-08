/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.factory;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.fptsimulation.GyroSimulationComponent;
import frc.robot.components.fptsimulation.SmarterSmartMotorSimulationComponent;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricWheelModule;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_MoveToPoseCommand;

/**
 * Add your docs here.
 */
public class OdometricSimulatedSwerveFactory {
    private final double MAX_SURFACE_SPEED = 2.5; // m/s
    private final double WHEEL_DIAMETER = 0.2; // m
    public OdometricSwerve makeSwerve(){
        return new OdometricSwerve(
            new GyroSimulationComponent(1),
            makeWheelModule(2, 1, 0.5, 0.5),
            makeWheelModule(4, 3, 0.5, -0.5),
            makeWheelModule(6, 5, -0.5, 0.5),
            makeWheelModule(8, 7, -0.5, -0.5)
        );
    }
    public OdometricWheelModule makeWheelModule(int angleId, int speedId, double forwardTranslation, double leftwardTranslation){
        var angleMotor = new SmarterSmartMotorSimulationComponent(angleId);
        var speedMotor = new SmarterSmartMotorSimulationComponent(speedId);
        return new OdometricWheelModule(
            angleMotor, 
            speedMotor, 
            new Translation2d(forwardTranslation, leftwardTranslation), 
            MAX_SURFACE_SPEED, 
            WHEEL_DIAMETER,
            1,
            1
        );
    }
    public OdometricSwerve_MoveToPoseCommand makeMoveToPoseCommand(String subsystem, OdometricSwerve swerve, Pose2d target){
        var leftwardController = new PIDController(1, 0, 0);
        SendableRegistry.add(leftwardController, subsystem+ "Leftward");
        leftwardController.setTolerance(0.1);
        SmartDashboard.putData(leftwardController);

        var forwardController = new PIDController(1, 0, 0);
        SendableRegistry.add(forwardController, subsystem+ "Forward");
        forwardController.setTolerance(0.1);
        SmartDashboard.putData(forwardController);

        var coutnerClockwardController = new PIDController(0.5, 0, 0);
        SendableRegistry.add(coutnerClockwardController, subsystem+ "Rotational");
        coutnerClockwardController.setTolerance(0.1);
        SmartDashboard.putData(coutnerClockwardController);

        return new OdometricSwerve_MoveToPoseCommand(
            swerve, 
            target, 
            leftwardController, 
            forwardController, 
            coutnerClockwardController);
    }
}
