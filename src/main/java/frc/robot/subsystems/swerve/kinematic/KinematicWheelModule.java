/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.kinematic;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;
import frc.robot.components.IAngularVelocitySetterComponent;

public class KinematicWheelModule extends SubsystemBase {


  protected IAngleSetterComponent angleSetterComponent;
  protected double angleRotsPerMotorRots;
  protected double driveRotsPerMotorRots;
  protected IAngularVelocitySetterComponent angularVelocitySetterComponent;
  protected Translation2d translationFromSwerveCenter;
  protected double maxSurfaceSpeed;
  protected double wheelDiameter;
  public KinematicWheelModule(IAngleSetterComponent angleSetterComponent, IAngularVelocitySetterComponent angularVelocitySetterComponent, Translation2d translationFromSwerveCenter, double maxSurfaceSpeed, double wheelDiameter, double angleRotsPerMotorRots, double driveRotsPerMotorRots) {
    this.angleSetterComponent = angleSetterComponent;
    this.angularVelocitySetterComponent = angularVelocitySetterComponent;
    this.translationFromSwerveCenter = translationFromSwerveCenter;
    this.maxSurfaceSpeed = maxSurfaceSpeed;
    this.wheelDiameter = wheelDiameter;
    this.angleRotsPerMotorRots = angleRotsPerMotorRots;
    this.driveRotsPerMotorRots = driveRotsPerMotorRots;
  }
  public void drive(SwerveModuleState state){
    angleSetterComponent.setAngle(state.angle.getRadians() / angleRotsPerMotorRots);
    angularVelocitySetterComponent.setAngularVelocity(state.speedMetersPerSecond / (wheelDiameter * Math.PI) * 2 * Math.PI / driveRotsPerMotorRots);
  }
  public Translation2d getTranslationFromSwerveCenter(){
    return translationFromSwerveCenter;
  }

  public double getMaxSurfaceSpeed() {
    return maxSurfaceSpeed;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
