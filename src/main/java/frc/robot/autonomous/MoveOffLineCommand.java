/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import frc.robot.subsystems.swerve.ISwerveOI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.vision.Vision;

public class MoveOffLineCommand extends CommandBase {
  private OdometricSwerve Swerve;
  private ISwerveOI oi;
  private PIDController xController, yController, wController;
  
  private Vision vision;
  public MoveOffLineCommand(OdometricSwerve Swerve, ISwerveOI oi, Vision vision) {
      addRequirements(Swerve);
      this.Swerve = Swerve;
      this.oi = oi;
      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
