/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

/**
 * @deprecated Use {@link frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand Autonomous_PreciseShootingCommand} instead.
 */
@Deprecated
public class Autonomous_StartShootingCommand extends CommandBase {
  /**
   * Creates a new Autonomous_StartShootingCommand.
   */
  private Indexer indexer;
  private Shooter shooter;
  private double topWheelSpeed, bottomWheelSpeed;
  public Autonomous_StartShootingCommand(Indexer indexer, Shooter shooter, double topWheelSpeed, double bottomWheelSpeed) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.topWheelSpeed = topWheelSpeed;
    this.bottomWheelSpeed = bottomWheelSpeed;
    addRequirements(indexer, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setSpeed(1);
    shooter.run(topWheelSpeed, bottomWheelSpeed);
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
