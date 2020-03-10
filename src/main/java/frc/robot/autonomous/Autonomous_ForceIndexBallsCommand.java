/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;

public class Autonomous_ForceIndexBallsCommand extends CommandBase {

  Indexer indexer;
  Intake intake;
  Arm arm;
  double indexerSpeed, intakeSpeed, armAngle;
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setSpeed(indexerSpeed);
    intake.setSpeed(intakeSpeed);
    arm.setAngle(armAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setSpeed(0);
    intake.setSpeed(0);
    arm.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Autonomous_ForceIndexBallsCommand(Indexer indexer, Intake intake, Arm arm, double indexerSpeed,
      double intakeSpeed, double armAngle) {
    this.indexer = indexer;
    this.intake = intake;
    this.arm = arm;
    this.indexerSpeed = indexerSpeed;
    this.intakeSpeed = intakeSpeed;
    this.armAngle = armAngle;
    addRequirements(indexer,intake,arm);
  }
}
