/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.kinematic.factory;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.components.IAngularVelocitySetterComponent;
import frc.robot.components.fptsimulation.SmartMotorSimulationComponent;
import frc.robot.components.virtual.VirtualGyroComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;

/**
 * Add your docs here.
 */
public class ThirteenWheelSwerveFactory {
    private class ShoehornedAngularVelocitySetter extends SmartMotorSimulationComponent implements IAngularVelocitySetterComponent{

        public ShoehornedAngularVelocitySetter(int motorId) {
            super(motorId);
        }

        @Override
        public void setAngularVelocity(double velocity) {
            setOutput(velocity);
        }

    }
    public KinematicSwerve makeSwerve(){
        return new KinematicSwerve(
            new VirtualGyroComponent(),
            makeWheelModule(1, 2, 0, 0),
            makeWheelModule(3, 4, -0.237, 0.432),
            makeWheelModule(5, 6, -0.331, 0.194),
            makeWheelModule(7,8, -0.475, -0.471),
            makeWheelModule(7+2, 8+2, -0.076, -0.444),
            makeWheelModule(9+2, 10+2, 0.356, -0.381),
            makeWheelModule(11+2, 12+2, 0.617, -0.023),
            makeWheelModule(13+2, 14+2, 0.544, 0.392),
            makeWheelModule(15+2, 16+2, 0.206, 0.454),
            makeWheelModule(17+2, 18+2, -0.245, -0.299),
            makeWheelModule(19+2, 20+2, 0.191, -0.205),
            makeWheelModule(21+2, 22+2, -0.097, 0.275),
            makeWheelModule(23+2, 24+2, 0.232, 0.195)
            );
    }
    private KinematicWheelModule makeWheelModule(int sid, int aid, double x, double y ){
        return new KinematicWheelModule(
            new ShoehornedAngularVelocitySetter(aid), 
            new ShoehornedAngularVelocitySetter(sid), 
            new Translation2d(x, y), 
            1,
            0.2,
            1,
            1);
    }
}
