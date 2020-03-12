/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.kinematic.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.utility.OutputRamper;

public class KinematicSwerve_RampedDriveCommand extends CommandBase {
  private KinematicSwerve swerve;
  private OutputRamper xRamper, yRamper, wRamper;
  private ISwerveOI oi;
  
  /**
   * Creates a new KinematicSwerve_RampedDriveCommand.
   */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xRamper.start();
    yRamper.start();
    wRamper.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xRamper.setSetpoint(oi.getX());
    yRamper.setSetpoint(oi.getY());
    wRamper.setSetpoint(oi.getZ());

    swerve.moveFieldCentric(xRamper.getOutput(), yRamper.getOutput(), wRamper.getOutput());
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

  public KinematicSwerve_RampedDriveCommand(KinematicSwerve swerve, OutputRamper xRamper, OutputRamper yRamper,
      OutputRamper wRamper, ISwerveOI oi) {
    this.swerve = swerve;
    this.xRamper = xRamper;
    this.yRamper = yRamper;
    this.wRamper = wRamper;
    this.oi = oi;
    addRequirements(swerve);
  }
  
}
