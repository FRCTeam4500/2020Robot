/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climbercarriage.commands;

import frc.robot.subsystems.climbercarriage.ClimberCarriage;
import frc.robot.subsystems.climbercarriage.IClimberCarriageOI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberCarriageCommand extends CommandBase {
  private ClimberCarriage climberCarriage;
  private IClimberCarriageOI oi;
  public ClimberCarriageCommand(ClimberCarriage climberCarriage, IClimberCarriageOI oi) {
    this.climberCarriage = climberCarriage;
    this.oi = oi;
    addRequirements(climberCarriage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberCarriage.setOutput(oi.getClimberCarriageOutput());
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
