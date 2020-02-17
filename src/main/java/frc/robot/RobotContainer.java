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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.IArmOI;
import frc.robot.subsystems.arm.commands.ArmRunCommand;
import frc.robot.subsystems.arm.factory.DefaultArmFactory;
import frc.robot.subsystems.indexer.command.Indexer;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements IArmOI {
  private boolean armActivated;
  private Arm arm;
  private DefaultArmFactory armFactory;
  private ArmRunCommand armRunCommand;
  private Joystick joystick;
  private JoystickButton button1;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    arm = armFactory.makeArm();
    armRunCommand = new ArmRunCommand(arm, this);
    arm.setDefaultCommand(armRunCommand);
    joystick = new Joystick(0);
    button1 = new JoystickButton(joystick,1);
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
    button1.whenPressed(new InstantCommand(() -> this.setArmActivated(true)));
    button1.whenReleased(new InstantCommand(() -> {this.setArmActivated(false);}));


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

  public void setArmActivated(boolean activated){
    armActivated = activated;
  }

  public boolean getArmActivated(){
    return armActivated;
  }
}
