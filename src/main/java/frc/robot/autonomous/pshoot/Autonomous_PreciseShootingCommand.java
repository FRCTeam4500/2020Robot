/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.pshoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class Autonomous_PreciseShootingCommand extends CommandBase {
  private Shooter shooter;
  private Indexer indexer;
  private IPreciseShootingOI oi;
  /**
   * Creates a new Autonomous_PreciseShootingCommand.
   */
  public Autonomous_PreciseShootingCommand(Shooter shooter, Indexer indexer, IPreciseShootingOI oi) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.oi = oi;
    addRequirements(shooter, indexer);
  }
  public Autonomous_PreciseShootingCommand(Shooter shooter, Indexer indexer, double topSpeed, double bottomSpeed, double coefficient, double threshold){
    this(shooter, indexer, new SpecificSpeedsPreciseShootingOI(topSpeed, bottomSpeed, coefficient, threshold));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var topSpeed = oi.getTopSpeed(); 
    var bottomSpeed = oi.getBottomSpeed();
    var k = oi.getCoefficient();
    var threshold = oi.getThreshold();

    shooter.run(topSpeed * k, bottomSpeed * k);
    if(shooter.atSpeeds(threshold)){
      indexer.setSpeed(1);
    }else{
      indexer.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disableMotors();
    indexer.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
