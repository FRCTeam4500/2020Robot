/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.utility.ExtendedMath;

public class OdometricSwerve_FollowTrajecoryCommand extends CommandBase {

  Trajectory.State[] states;
  int currentStateIndex = 0;
  
  private Trajectory.State target;

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
  public OdometricSwerve_FollowTrajecoryCommand(OdometricSwerve swerve, PIDController controller, Trajectory.State... states) {
    this.swerve = swerve;
    this.controller = controller;
    this.states = states;
    addRequirements(swerve);
  }
  public OdometricSwerve_FollowTrajecoryCommand(OdometricSwerve swerve, PIDController controller, Trajectory trajectory){
    this(swerve, controller, trajectory.getStates().toArray(Trajectory.State[]::new));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentStateIndex = 0;
    initializeNewTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var output = controller.calculate(getProjectedOffsetFromTarget())+target.velocityMetersPerSecond;
    var direction = ExtendedMath.normalize(getOffsetToTarget());
    swerve.moveFieldCentric(direction.getX() * output, direction.getY() * output, 0);
    if(controller.atSetpoint() && currentStateIndex + 1 < states.length){
      currentStateIndex++;
      initializeNewTarget();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  private void initializeNewTarget(){
    target = states[currentStateIndex];
    axis = getOffsetToTarget();
    controller.reset();
    controller.setSetpoint(0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint() && currentStateIndex + 1 >= states.length;
  }
  private Translation2d getOffsetFromTarget(){
    return swerve.getCurrentPose().getTranslation().minus(target.poseMeters.getTranslation());
  }
  private Translation2d getOffsetToTarget(){
    return target.poseMeters.getTranslation().minus(swerve.getCurrentPose().getTranslation());
  }
  private double getScalarProjectionOntoTargetAxis(Translation2d vector){
    return ExtendedMath.scalarProjectionOf(vector, axis);
  }
  private double getProjectedOffsetFromTarget(){
    return getScalarProjectionOntoTargetAxis(getOffsetFromTarget());
  }
}
