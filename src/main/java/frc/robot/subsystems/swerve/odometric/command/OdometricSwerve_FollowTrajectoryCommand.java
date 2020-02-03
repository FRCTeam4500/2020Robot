/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.*;

public class OdometricSwerve_FollowTrajectoryCommand extends CommandBase {

  Trajectory trajectory;
  OdometricSwerve swerve;
  RamseteController controller;
  double startTime;
  
  /**
   * Creates a new OdometricSwerve_ProfiledMoveToPoseCommand.
   */
  public OdometricSwerve_FollowTrajectoryCommand(OdometricSwerve swerve, Trajectory trajectory, RamseteController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.trajectory = trajectory;
    this.controller = controller;
    addRequirements(swerve);
  }

  /**
   * @return the controller
   */
  public RamseteController getController() {
    return controller;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var speeds = controller.calculate(swerve.getCurrentPose(), trajectory.sample(Timer.getFPGATimestamp() - startTime));
    swerve.moveRobotCentric(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
