/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IVisionComponent;

public class VisionSubsystem extends SubsystemBase {

  private IVisionComponent vision;
  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem(IVisionComponent vision) {
    this.vision = vision;
    vision.setPipeline(4);
  }

  public double getHorizontalOffset(){
    return vision.getHorizontalOffsetFromCrosshair();
  }
  public double getVerticalOffset(){
    return vision.getVerticalOffsetFromCrosshair();
  }
  public double getSkew(){
    return vision.getSkew();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
