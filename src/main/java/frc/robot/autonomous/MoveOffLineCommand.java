/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_MoveToPoseCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.utility.ExtendedMath;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_MoveToPoseCommand;
import edu.wpi.first.networktables.NetworkTableEntry;
public class MoveOffLineCommand extends CommandBase {
  private OdometricSwerve swerve;
  private VisionSubsystem vision;
  private ISwerveOI oi;
  
  public MoveOffLineCommand(VisionSubsystem vision, OdometricSwerve swerve, ISwerveOI oi) {
    addRequirements(vision);
    addRequirements(swerve);
    
    this.vision = vision;
    this.swerve = swerve;
    this.oi = oi;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ExtendedMath.clamp(-1, 1, -vision.getHorizontalOffset() * 0.05);
    
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
}
