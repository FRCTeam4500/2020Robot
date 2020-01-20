/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.kinematic;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IGyroComponent;

public class KinematicSwerve extends SubsystemBase {

  protected SwerveDriveKinematics kinematics;
  protected KinematicWheelModule[] wheelModules;
  protected double lowestMaximumWheelSpeed;

  protected IGyroComponent gyro;
  /**
   * Creates a new KinematicSwerve.
   */
  public KinematicSwerve(IGyroComponent gyro, KinematicWheelModule... wheelModules) {
    this.wheelModules = wheelModules;
    this.gyro = gyro;
    
    kinematics = new SwerveDriveKinematics(getTranslations(wheelModules));

    lowestMaximumWheelSpeed = getLowestMaximumWheelModuleSpeeds(wheelModules);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveRobotCentric(double xSpeed, double ySpeed, double wSpeed){
    var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, wSpeed);  
    moveRobotCentric(chassisSpeeds);
  }
  public void moveRobotCentric(ChassisSpeeds chassisSpeeds){
    var states = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, lowestMaximumWheelSpeed);

    for(int i = 0;i<wheelModules.length;i++){
      wheelModules[i].drive(states[i]);
    }  
  }
  public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, double robotAngle){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, new Rotation2d(robotAngle));
  }
  public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle){
    
    var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, wSpeed, robotAngle);
    moveRobotCentric(chassisSpeeds);
  }
  public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, gyro.getAngle());
  }

  private Translation2d[] getTranslations(KinematicWheelModule[] wheelModules){
    var translations = new Translation2d[wheelModules.length];
    for(int i = 0;i < wheelModules.length;i++){
      translations[i] = wheelModules[i].getTranslationFromSwerveCenter();
    }
    return translations;
  }
  private double getLowestMaximumWheelModuleSpeeds(KinematicWheelModule[] wheelModules){
    var lowestMaxSpeed = Double.MAX_VALUE;
    for(var module : wheelModules){
      lowestMaxSpeed = Math.min(module.getMaxSurfaceSpeed(), lowestMaxSpeed);
    }
    return lowestMaxSpeed;
  }
}
