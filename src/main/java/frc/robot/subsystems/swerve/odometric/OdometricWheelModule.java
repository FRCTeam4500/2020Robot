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
import frc.robot.components.ISpeedGetterComponent;
import frc.robot.components.ISpeedSetterComponent;
import frc.robot.subsystems.swerve.kinematic.KinematicWheelModule;

/**
 * Add your docs here.
 */
public class OdometricWheelModule extends KinematicWheelModule {

    protected IAngleGetterComponent angleGetterComponent;
    protected ISpeedGetterComponent speedGetterComponent;
    public OdometricWheelModule(IAngleSetterComponent angleSetterComponent, ISpeedSetterComponent speedSetterComponent,
            Translation2d translationFromSwerveCenter, double maxSurfaceSpeed, IAngleGetterComponent angleGetterComponent, ISpeedGetterComponent speedGetterComponent) {
        super(angleSetterComponent, speedSetterComponent, translationFromSwerveCenter, maxSurfaceSpeed);
        this.angleGetterComponent = angleGetterComponent;
        this.speedGetterComponent = speedGetterComponent;
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(speedGetterComponent.getSpeed()*getMaxSurfaceSpeed(), new Rotation2d(angleGetterComponent.getAngle()));
    }
}
