/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.utility.ExtendedMath;

public class OdometricSwerve_MoveToTranslationCommand extends CommandBase {
  private Translation2d target;
  private double speedAtTarget;

  private PIDController controller;
  private OdometricSwerve swerve;

  Translation2d axis;
  
  /**
   * @return the controller
   */
  public PIDController getController() {
    return controller;
  }
  /**
   * Creates a new OdometricSwerve_MoveToTranslationCommand.
   */
  public OdometricSwerve_MoveToTranslationCommand(OdometricSwerve swerve, Translation2d target, double speedAtTarget, PIDController controller) {
    this.swerve = swerve;
    this.target = target;
    this.speedAtTarget = speedAtTarget;
    this.controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    axis = getOffsetToTarget();
    controller.reset();
    controller.setSetpoint(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var output = controller.calculate(getProjectedOffsetFromTarget())+speedAtTarget;
    var direction = ExtendedMath.normalize(getOffsetToTarget());
    swerve.moveFieldCentric(direction.getX() * output, direction.getY() * output, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
  private Translation2d getOffsetFromTarget(){
    return swerve.getCurrentPose().getTranslation().minus(target);
  }
  private Translation2d getOffsetToTarget(){
    return target.minus(swerve.getCurrentPose().getTranslation());
  }
  private double getScalarProjectionOntoTargetAxis(Translation2d vector){
    return ExtendedMath.scalarProjectionOf(vector, axis);
  }
  private double getProjectedOffsetFromTarget(){
    return getScalarProjectionOntoTargetAxis(getOffsetFromTarget());
  }
}
