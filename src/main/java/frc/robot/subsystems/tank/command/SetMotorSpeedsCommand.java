/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.tank.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.tank.Tank;

public class SetMotorSpeedsCommand extends CommandBase {

  private Tank tank;
  private double leftSpeed, rightSpeed;
  /**
   * Creates a new SetMotorSpeedsCommand.
   */
  public SetMotorSpeedsCommand(Tank tank, double leftSpeed, double rightSpeed) {

    this.tank = tank;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(tank);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tank.drive(leftSpeed, rightSpeed);
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
    return true;
  }
}
