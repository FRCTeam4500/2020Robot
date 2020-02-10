/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.spinner.command;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ColorMatcherCommand extends CommandBase {

  ColorSensorV3 sensor;
  private ColorMatch matcher;

  private final Color kBlueTarget = ColorMatch.makeColor(0.150, 0.440, 0.370);
  private final Color kGreenTarget = ColorMatch.makeColor(0.202, 0.544, 0.251);
  private final Color kRedTarget = ColorMatch.makeColor(0.480, 0.373, 0.175);
  private final Color kYellowTarget = ColorMatch.makeColor(0.318, 0.557, 0.130);
  /**
   * Creates a new ColorMatcherCommand.
   */
  public ColorMatcherCommand(ColorSensorV3 sensor, ColorMatch matcher) {
    this.matcher = matcher;
    this.sensor = sensor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    matcher.addColorMatch(kBlueTarget);
    matcher.addColorMatch(kGreenTarget);
    matcher.addColorMatch(kRedTarget);
    matcher.addColorMatch(kYellowTarget);   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var match = matcher.matchColor(sensor.getColor());
    
    String colorString =  "Unknown";

    if(match != null)
    {
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    
  }
  SmartDashboard.putString("Color", colorString);
    var color = (sensor.getColor());
    SmartDashboard.putNumber("RedValue",color.red);
    SmartDashboard.putNumber("BlueValue",color.blue);
    SmartDashboard.putNumber("GreenValue",color.green);
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
