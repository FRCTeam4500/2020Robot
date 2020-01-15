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
  ISpeedSetterComponent fl, fr, bl, br;
  /**
   * Creates a new Tank.
   */
  public Tank(ISpeedSetterComponent fl, ISpeedSetterComponent fr, ISpeedSetterComponent bl, ISpeedSetterComponent br) {
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;
  }

  public void drive(double leftSpeed, double rightSpeed){
    fl.setSpeed(leftSpeed);
    bl.setSpeed(leftSpeed);

    fr.setSpeed(rightSpeed);
    br.setSpeed(rightSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
