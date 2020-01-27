/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.command.ShootAtAngleCommand;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.command.SetTurretAngleCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements ITurretOI, IShooterOI {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  private double turretAngle;
  private double shooterAngle;
  private boolean turretActive;
  private Shooter shooter;
  private ShootAtAngleCommand shootCommand;
  private Turret turret;
  private SetTurretAngleCommand setTurretAngleCommand;
  public RobotContainer() {
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

  public double getTurretAngle(){
    return this.turretAngle;
  }

  public double getShooterAngle(){
      return this.shooterAngle;
  }

  public boolean getTurretActive(){
      return this.turretActive;
  }
}
