/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.tank.Tank;
import frc.robot.subsystems.tank.command.TankDriveCommand;
import frc.robot.subsystems.tank.command.TankSpinCommand;
import frc.robot.subsystems.tank.factory.TankFactory;
import frc.robot.subsystems.tank.ITankOI;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements ITankOI {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  TankFactory tankFactory;
  Tank tank;
  TankDriveCommand tankDriveCommand;
  TankSpinCommand tankSpinCommand;
  Joystick joystick;
  JoystickButton button1;


  public RobotContainer() {
    this.tankFactory = new TankFactory();
    this.tank = tankFactory.makeTank();
    this.tankDriveCommand = new TankDriveCommand(tank, this);
    this.tankSpinCommand = new TankSpinCommand(tank, this);
    this.joystick = new Joystick(0);
    tank.setDefaultCommand(tankDriveCommand);
    
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    this.button1 = new JoystickButton(joystick, 0);
    button1.whenPressed(tankSpinCommand);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public double getX(){
    return joystick.getX();
  }
  
  public double getY(){
    return joystick.getY();
  }
}
