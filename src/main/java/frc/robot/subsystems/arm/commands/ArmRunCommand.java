/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.arm.commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmMap;
import frc.robot.subsystems.arm.IArmOI;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmRunCommand extends CommandBase {
  private Arm arm;
  private IArmOI oi;

  //min/max are 0 and -3600 sensor units
//plug in correct values to min and max
  public ArmRunCommand(Arm arm, IArmOI oi) {
    this.arm = arm;
    this.oi = oi;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (oi.getArmActive() == true){
      arm.setAngle(ArmMap.ARM_MAX_VALUE);
    }
    else{
      arm.setAngle(ArmMap.ARM_MIN_VALUE);
    }

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
