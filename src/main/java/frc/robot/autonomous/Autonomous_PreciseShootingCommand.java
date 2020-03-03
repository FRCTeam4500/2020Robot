/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class Autonomous_PreciseShootingCommand extends CommandBase {
  private Shooter shooter;
  private Indexer indexer;
  private boolean runBothMotors;
  /**
   * Creates a new Autonomous_PreciseShootingCommand.
   */
  public Autonomous_PreciseShootingCommand(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    addRequirements(shooter, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  public void createSmartDashboardEntries(){
    SmartDashboard.putNumber("topSpeed", -0);
    SmartDashboard.putNumber("bottomSpeed", -0);
    SmartDashboard.putNumber("coefficient", 1);
    SmartDashboard.putNumber("threshold", 45);
    runBothMotors = false;

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var topSpeed = SmartDashboard.getNumber("topSpeed", 0.0);
    var bottomSpeed = SmartDashboard.getNumber("bottomSpeed", 0.0);
    var k = SmartDashboard.getNumber("coefficient", 1.0);
    var threshold = SmartDashboard.getNumber("threshold", 0.0);

    /*if(runBothMotors){
      shooter.run(topSpeed * k, bottomSpeed * k);
    } else {
      shooter.run(topSpeed * k, 0);
      if(shooter.topAtSpeed(threshold)){
        runBothMotors = true;
      }
    }*/

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