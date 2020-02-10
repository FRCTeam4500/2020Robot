/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.indexer.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.ISpeedSetterComponent;
import frc.robot.components.Sensor;
import frc.robot.components.hardware.VictorSPComponent;

public class IndexBallsCommand extends CommandBase {
  ISpeedSetterComponent indexMotor = new VictorSPComponent(0);
  ISpeedSetterComponent intakeMotor = new ISpeedSetterComponent(){
  
    @Override
    public void setSpeed(double speed) {
      SmartDashboard.putBoolean("Run Intake Motor", speed != 0);
    }
  };
  
  Sensor sensor1 = new Sensor("Sensor1"), 
      sensor0 = new Sensor("Sensor0"),
      sensor2 = new Sensor("Sensor2"),
      sensor3 = new Sensor("Sensor3"),
      sensor4 = new Sensor("Sensor4"),
      sensor5 = new Sensor("Sensor5");

  boolean indexingBall = false;
  int ballCount = 0;



  

  /**
   * Creates a new Indexer.
   */
  public IndexBallsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  double motorSpeed = -0.75;
  @Override
  public void execute() {
    if(sensor0.registersBall()){
      indexMotor.setSpeed(motorSpeed);
      intakeMotor.setSpeed(0);
      indexingBall = true;
    }
    else if(sensor1.registersBall() ||
    sensor2.registersBall() ||
    sensor3.registersBall() ||
    sensor4.registersBall() ||
    sensor5.registersBall()){
      indexMotor.setSpeed(0);
      intakeMotor.setSpeed(motorSpeed);
      if(indexingBall){
        indexingBall = false;
        ballCount++;
      }
    }
    SmartDashboard.putNumber("ball count", ballCount);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeMotor.setSpeed(0.0);
    indexMotor.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
