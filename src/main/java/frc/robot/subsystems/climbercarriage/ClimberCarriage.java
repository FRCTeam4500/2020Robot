/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climbercarriage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IOutputSetterComponent;

public class ClimberCarriage extends SubsystemBase {
  IOutputSetterComponent motor1;
  public ClimberCarriage(IOutputSetterComponent motor1) {
    this.motor1 = motor1;
  }

  public void run(double speed){
this.motor1.setOutput(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
