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
  protected double currentGyroZero = 0.0;
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
  /**
   * A wrapper for {@link #moveRobotCentric(ChassisSpeeds)}.
   * @param xSpeed Forward speed in meters/second
   * @param ySpeed Leftward speed in meters/second
   * @param wSpeed Counterclockwise rotational speed in radians/second
   */
  public void moveRobotCentric(double xSpeed, double ySpeed, double wSpeed){
    var chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, wSpeed);  
    moveRobotCentric(chassisSpeeds);
  }
  /**
   * Move the swerve drive relative to itself using the desired {@link ChassisSpeeds}.
   * @param chassisSpeeds the target chassis speeds for the swerve drive.
   * @see #moveRobotCentric(double, double, double)
   */
  public void moveRobotCentric(ChassisSpeeds chassisSpeeds){
    moveRobotCentric(chassisSpeeds, new Translation2d());
  }
  public void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
    var states = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, lowestMaximumWheelSpeed);

    for(int i = 0;i<wheelModules.length;i++){
      wheelModules[i].drive(states[i]);
    }   
  }
  /**
   * A wrapper for {@link #moveAngleCentric(double, double, double, Rotation2d)}
   */
  public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, double robotAngle){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, new Rotation2d(robotAngle));
  }
  /**
   * Move the swerve drive relative to an angle. This angle is usually the gyroscope reading. When the angle is zero, the swerve drive is assumed to face the positive X direction, and positive Y is directly to the left of the swerve drive.
   * @param xSpeed the forward speed to move when angle is zero in meters/second
   * @param ySpeed the leftward speed to move when angle is zero in meters/second
   * @param wSpeed the counterclockward speed to rotate in radians/second
   * @param robotAngle the angle of the robot relative to the described coordinate system
   */
  public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, robotAngle, new Translation2d());
  }
  public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle, Translation2d centerOfRotation){
    var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, wSpeed, robotAngle);
    moveRobotCentric(chassisSpeeds, centerOfRotation);
  }
  public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, gyro.getAngle() - currentGyroZero);
  }
  public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed, Translation2d centerOfRotation){
    moveAngleCentric(xSpeed, ySpeed, wSpeed, new Rotation2d(gyro.getAngle() - currentGyroZero),centerOfRotation);
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
  protected void resetGyro(){
    gyro.reset();
  }

  public double getGyroAngle(){
    return gyro.getAngle() - currentGyroZero;
  }

  public void resetRobotAngle(){
    currentGyroZero = gyro.getAngle();
  }
  public void resetRobotAngle(double angle){
    currentGyroZero = gyro.getAngle() - angle;
  }
}
