/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.indexer.Indexer;

public class Autonomous_IndexBallsCommand extends CommandBase {

  Indexer indexer;
  double indexerSpeed;
  double intakeSpeed;
  Intake intake;

  /**
   * Creates a new Indexer.
   */
  public Autonomous_IndexBallsCommand(Indexer indexer, Intake intake, double motorSpeed, double intakeSpeed) {
    this.indexer = indexer;
    this.indexerSpeed = motorSpeed;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    addRequirements(indexer, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSpeed(intakeSpeed);
    indexer.setSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexer.sensor0RegistersBall() && !indexer.sensor5RegistersBall()){
      indexer.setSpeed(indexerSpeed);
      intake.setSpeed(0);
    }
    else if(indexer.sensor1RegistersBall() ||
    indexer.sensor2RegistersBall() ||
    indexer.sensor3RegistersBall() ||
    indexer.sensor4RegistersBall() ||
    indexer.sensor5RegistersBall()){
      indexer.setSpeed(0);
      intake.setSpeed(intakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSpeed(0.0);
    intake.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
