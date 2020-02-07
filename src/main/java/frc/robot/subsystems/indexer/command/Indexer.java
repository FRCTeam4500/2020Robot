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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Indexer extends CommandBase {
  private NetworkTableInstance nt;
  private NetworkTable table;
  private NetworkTableEntry Sensor0;
  private NetworkTableEntry Sensor1;
  private NetworkTableEntry Sensor2;

  private NetworkTableEntry Sensor3;
  private NetworkTableEntry Sensor4;
  private NetworkTableEntry Sensor5;


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
  Sensor sensor0 = new Sensor();
  Sensor sensor1 = new Sensor();
  boolean indexingBall = false;
  int ballCount = 0;
  Sensor sensor2 = new Sensor();
  Sensor sensor3 = new Sensor();
  Sensor sensor4 = new Sensor();
  Sensor sensor5 = new Sensor();


  

  /**
   * Creates a new Indexer.
   */
  public Indexer(NetworkTable table) {
    nt = NetworkTableInstance.getDefault();
    table = nt.getTable("SmartDashboard");
    Sensor0 = table.getEntry("Sensor0");
    Sensor1 = table.getEntry("Sensor1");
    Sensor2 = table.getEntry("Sensor2");
    Sensor3 = table.getEntry("Sensor3");
    Sensor4 = table.getEntry("Sensor4");
    Sensor5 = table.getEntry("Sensor5");
  }
  
  public double getSensor0() {
    return Sensor0.getDouble(0);
  }
  public double getSensor1() {
    return Sensor1.getDouble(0);
  }
  public double getSensor2() {
    return Sensor2.getDouble(0);
  }
  public double getSensor3() {
    return Sensor3.getDouble(0);
  }
  public double getSensor4() {
    return Sensor4.getDouble(0);
  }
  public double getSensor5() {
    return Sensor5.getDouble(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sensor0.registersBall()){
      indexMotor.setSpeed(1);
      intakeMotor.setSpeed(0);
      indexingBall = true;
      if(!sensor1.registersBall()) {
          if(sensor1.registersBall()) {
            indexMotor.setSpeed(0);
          }
      }
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
