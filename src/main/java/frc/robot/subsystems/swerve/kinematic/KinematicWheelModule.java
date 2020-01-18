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
import frc.robot.components.ISpeedSetterComponent;

public class KinematicWheelModule extends SubsystemBase {


  private IAngleSetterComponent angleSetterComponent;
  private ISpeedSetterComponent speedSetterComponent;
  private Translation2d translationFromSwerveCenter;
  private double maxSurfaceSpeed;
  public KinematicWheelModule(IAngleSetterComponent angleSetterComponent, ISpeedSetterComponent speedSetterComponent, Translation2d translationFromSwerveCenter, double maxSurfaceSpeed) {
    this.angleSetterComponent = angleSetterComponent;
    this.speedSetterComponent = speedSetterComponent;
    this.translationFromSwerveCenter = translationFromSwerveCenter;
    this.maxSurfaceSpeed = maxSurfaceSpeed;
  }
  public void drive(SwerveModuleState state){
    angleSetterComponent.setAngle(state.angle.getRadians());
    speedSetterComponent.setSpeed(state.speedMetersPerSecond/maxSurfaceSpeed);
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
