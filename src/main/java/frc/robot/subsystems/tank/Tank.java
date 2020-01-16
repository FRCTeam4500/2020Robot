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
  /**
   * Creates a new Tank.
   */
    ISpeedSetterComponent l1;
    ISpeedSetterComponent l2;
    ISpeedSetterComponent r1;
    ISpeedSetterComponent r2;

  public Tank(ISpeedSetterComponent l1, ISpeedSetterComponent l2, ISpeedSetterComponent r1, ISpeedSetterComponent r2) {
    this.l1 = l1;
    this.l2 = l2;
    this.r1 = r1;
    this.r2 = r2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double leftSpeed, double rightSpeed){
    l1.setSpeed(leftSpeed);
    l2.setSpeed(leftSpeed);
    r1.setSpeed(rightSpeed);
    r2.setSpeed(rightSpeed);
  }
}
