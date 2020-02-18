/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.components.IAngleGetterComponent;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.IAngularVelocityGetterComponent;
import frc.robot.components.IAngularVelocitySetterComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;

/**
 * Add your docs here.
 */
public class OdometricWheelModule extends KinematicWheelModule {

    protected IAngleGetterComponent angleGetterComponent;
    protected IAngularVelocityGetterComponent angularVelocityGetterComponent;
    
    public OdometricWheelModule(IAngleSetterComponent angleSetterComponent, IAngularVelocitySetterComponent angularVelocitySetterComponent,
            Translation2d translationFromSwerveCenter, double maxSurfaceSpeed, IAngleGetterComponent angleGetterComponent, IAngularVelocityGetterComponent angularVelocityGetterComponent,double wheelDiameter, double angleRotsPerMotorRots, double driveRotsPerMotorRots) {
        super(angleSetterComponent, angularVelocitySetterComponent, translationFromSwerveCenter, maxSurfaceSpeed, wheelDiameter, angleRotsPerMotorRots, driveRotsPerMotorRots);
        this.angleGetterComponent = angleGetterComponent;
        this.angularVelocityGetterComponent = angularVelocityGetterComponent;
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(angularVelocityGetterComponent.getAngularVelocity() / 2 / Math.PI * Math.PI *wheelDiameter, new Rotation2d(angleGetterComponent.getAngle()));
    }
}
