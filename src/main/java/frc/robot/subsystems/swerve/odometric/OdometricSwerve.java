/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.components.IGyroComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;

/**
 * Add your docs here.
 */
public class OdometricSwerve extends KinematicSwerve {

    SwerveDriveOdometry odometry;
    OdometricWheelModule[] odometricWheelModules;
    public OdometricSwerve(IGyroComponent gyro, OdometricWheelModule... wheelModules) {
        super(gyro, wheelModules);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getAngle()));
        odometricWheelModules = wheelModules;
    }
    public Pose2d updateOdometry(){
        return odometry.update(new Rotation2d(gyro.getAngle()), getSwerveModuleStates());
    }
    private SwerveModuleState[] getSwerveModuleStates(){
        var states = new SwerveModuleState[wheelModules.length];
        for(int i = 0;i<states.length;i++){
            states[i] = odometricWheelModules[i].getState();
        }
        return states;
    }
}
