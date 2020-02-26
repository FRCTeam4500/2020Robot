/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;

public class OdometricSwerve_AdvancedFollowTrajectoryUsingOffsetCommand extends CommandBase {
  /**
   * Creates a new OdometricSwerve_AdvancedFollowTrajectoryUsingOffsetCommand.
   */
  private OdometricSwerve swerve;
  private AdvancedSwerveController controller;
  private Transform2d localOffset;


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset(getOffsetSwervePose().getTranslation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var output = controller.calculateFieldCentricChassisSpeeds(getOffsetSwervePose());
    swerve.moveFieldCentric(output.vxMetersPerSecond, output.vyMetersPerSecond, output.omegaRadiansPerSecond, localOffset.getTranslation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atFinalPose(getOffsetSwervePose());
  }
  private Pose2d getOffsetSwervePose(){
    return swerve.getCurrentPose().plus(localOffset);
  }

  public OdometricSwerve_AdvancedFollowTrajectoryUsingOffsetCommand(OdometricSwerve swerve,
      AdvancedSwerveController controller, Transform2d localOffset) {
    this.swerve = swerve;
    this.controller = controller;
    this.localOffset = localOffset;
    addRequirements(swerve);
  }
}
