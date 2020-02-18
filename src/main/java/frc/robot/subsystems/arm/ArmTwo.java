/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmTwo extends SubsystemBase {
  TalonSRX srx;
  /**
   * Creates a new ArmTwo.
   */
  public ArmTwo() {
    srx = new TalonSRX(8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setAngle(int angle){
    srx.set(ControlMode.Position, angle);
  }
}
