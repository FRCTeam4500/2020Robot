/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.tank.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.ITankOI;
import frc.robot.subsystems.tank.Tank;

public class DriveTankCommand extends CommandBase {
  private Tank tank;
  private ITankOI oi;
  /**
   * Creates a new DriveTankCommand.
   */
  public DriveTankCommand(Tank tank, ITankOI oi) {
    this.tank = tank;
    this.oi = oi;
    addRequirements(tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var leftSpeed  =oi.getLeftSpeed();
    var rightSpeed = oi.getRightSpeed();
    tank.drive(leftSpeed, rightSpeed);
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
