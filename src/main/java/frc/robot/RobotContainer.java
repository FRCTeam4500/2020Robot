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
import frc.robot.subsystems.tank.ITankOI;
import frc.robot.subsystems.tank.Tank;
import frc.robot.subsystems.tank.command.DriveTankCommand;
import frc.robot.subsystems.tank.command.SetMotorSpeedsCommand;
import frc.robot.subsystems.tank.factory.DefaultTankFactory;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements ITankOI {

  private SetMotorSpeedsCommand command;
  private Tank tank;

  private Joystick joystick = new Joystick(0);
  private JoystickButton button = new JoystickButton(joystick, 1);
  private JoystickButton button2 = new JoystickButton(joystick, 2);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    tank = new DefaultTankFactory().makeTank();
    command = new SetMotorSpeedsCommand(tank, 1.0, -1.0);
    var command2 = new SetMotorSpeedsCommand(tank, 0, 0);
    
    var driveCommand = new DriveTankCommand(tank, this);
    tank.setDefaultCommand(driveCommand);

    button.whenPressed(command);

    button2.whenPressed(command2);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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

  @Override
  public double getLeftSpeed() {
    return joystick.getX();
  }

  @Override
  public double getRightSpeed() {
    return joystick.getY();
  }
}
