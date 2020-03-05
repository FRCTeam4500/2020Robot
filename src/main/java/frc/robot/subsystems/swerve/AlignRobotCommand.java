/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.components.IVisionComponent;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AlignRobotCommand extends CommandBase implements IVisionComponent{

  private VisionSubsystem vision;
  private SmartDashboard dashboard;
  //TODO actually make the stupid stinky thingie
  public AlignRobotCommand(VisionSubsystem vision) {
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO actually put something here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean hasValidTargets() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double getHorizontalOffsetFromCrosshair() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getVerticalOffsetFromCrosshar() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getTargetArea() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getSkew() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void setCameraMode(CameraMode mode) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setPipeline(int index) {
    // TODO Auto-generated method stub

  }
}
