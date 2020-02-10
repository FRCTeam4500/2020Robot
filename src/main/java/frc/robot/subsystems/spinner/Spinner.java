/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.spinner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngleSetterComponent;

public class Spinner extends SubsystemBase {
  IAngleSetterComponent motor;
  public Spinner(IAngleSetterComponent motor) {
    this.motor = motor;
  }

  public void setAngle(double angle){
    motor.setAngle(angle);
  }
  
  @Override
  public void periodic() {
  }
}
