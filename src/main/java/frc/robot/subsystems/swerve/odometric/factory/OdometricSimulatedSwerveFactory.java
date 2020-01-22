/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.factory;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.components.fptsimulation.SmarterSmartMotorSimulationComponent;
import frc.robot.components.virtual.VirtualGyroComponent;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricWheelModule;

/**
 * Add your docs here.
 */
public class OdometricSimulatedSwerveFactory {
    private final double MAX_SURFACE_SPEED = 0.0; // m/s
    private final double WHEEL_DIAMETER = 0.0; // m
    public OdometricSwerve makeSwerve(){
        return new OdometricSwerve(
            new VirtualGyroComponent(),
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
            angleMotor, 
            speedMotor, 
            WHEEL_DIAMETER
        );
    }
}
