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

public class Indexer extends CommandBase {
  ISpeedSetterComponent indexMotor = new ISpeedSetterComponent(){
  
    @Override
    public void setSpeed(double speed) {
      SmartDashboard.putBoolean("Run Index Motor", speed != 0);
    }
  };
  ISpeedSetterComponent intakeMotor = new ISpeedSetterComponent(){
  
    @Override
    public void setSpeed(double speed) {
      SmartDashboard.putBoolean("Run Intake Motor", speed != 0);
    }
  };
  Sensor sensor1 = new Sensor();
  boolean indexingBall = false;
  int ballCount = 0;
  Sensor sensor2 = new Sensor();
  Sensor sensor3 = new Sensor();
  Sensor sensor4 = new Sensor();
  Sensor sensor5 = new Sensor();
  Sensor sensor6 = new Sensor();

  

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sensor1.registersBall()){
      indexMotor.setSpeed(1);
      intakeMotor.setSpeed(0);
      indexingBall = true;
      if(!sensor2.registersBall()) {
          if(sensor2.registersBall()) {
            indexMotor.setSpeed(0);
          }
      }
      if(!sensor3.registersBall()) {
        if(sensor3.registersBall()) {
          indexMotor.setSpeed(0);
        }
      }

      if(!sensor4.registersBall()) {
        if(sensor4.registersBall()) {
          indexMotor.setSpeed(0);
        }
      }
      if(!sensor5.registersBall()) {
        if(sensor5.registersBall()) {
          indexMotor.setSpeed(0);
        }
      }
    }else{
      indexMotor.setSpeed(0);
      intakeMotor.setSpeed(1);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
