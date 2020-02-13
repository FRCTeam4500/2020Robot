/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;

public class OdometricSwerve_MoveToPoseCommand extends CommandBase {
  private OdometricSwerve swerve;
  private Pose2d target;
  private PIDController leftwardController, forwardController, counterClockwardController;
  /**
   * Creates a new OdometricSwerve_MoveToPoseCommand.
   */
  public OdometricSwerve_MoveToPoseCommand(OdometricSwerve swerve, Pose2d target, PIDController leftwardController, PIDController forwardController, PIDController counterClockwardController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.target = target;
    this.leftwardController = leftwardController;
    this.forwardController = forwardController;
    this.counterClockwardController = counterClockwardController;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftwardController.setSetpoint(target.getTranslation().getY());
    leftwardController.reset();
    forwardController.setSetpoint(target.getTranslation().getX());
    forwardController.reset();
    counterClockwardController.setSetpoint(target.getRotation().getRadians());
    counterClockwardController.enableContinuousInput(0, 2 * Math.PI);
    counterClockwardController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = swerve.getCurrentPose();
    var forwardOutput = forwardController.calculate(currentPose.getTranslation().getX());
    var leftwardOutput = leftwardController.calculate(currentPose.getTranslation().getY());
    var counterClockwardOutput = counterClockwardController.calculate(currentPose.getRotation().getRadians());
    swerve.moveFieldCentric(forwardOutput, leftwardOutput, counterClockwardOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftwardController.atSetpoint() && forwardController.atSetpoint() && counterClockwardController.atSetpoint();
  }
  public PIDController getForwardController(){
    return forwardController;
  }
  public PIDController getLeftwardController(){
    return leftwardController;
  }
  public PIDController getCounterClockwardController(){
    return counterClockwardController;
  }
}
