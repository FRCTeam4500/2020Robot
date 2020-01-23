/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.tank;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISpeedSetterComponent;

public class Tank extends SubsystemBase {
 
  private ISpeedSetterComponent frontLeft, frontRight, backLeft, backRight;

  public Tank(ISpeedSetterComponent frontLeft, ISpeedSetterComponent frontRight, ISpeedSetterComponent backLeft, ISpeedSetterComponent backRight) {
    this.frontLeft = frontLeft;
    this.frontRight = frontRight;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }

  public void drive(double leftSpeed, double rightSpeed){
    frontLeft.setSpeed(leftSpeed);
    backLeft.setSpeed(leftSpeed);

   frontRight.setSpeed(rightSpeed);
   backRight.setSpeed(rightSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
