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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.command.ShootStraightCommand;
import frc.robot.subsystems.shooter.factory.DefaultShooterFactory;
import frc.robot.subsystems.shooter.factory.IShooterFactory;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.command.SetTurretAngleCommand;
import frc.robot.subsystems.turret.factory.DefaultTurretFactory;
import frc.robot.subsystems.turret.factory.ITurretFactory;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RegularRobotContainer implements ITurretOI, IShooterOI, IRobotContainer {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  private double turretAngle;
  private double shooterAngle;
  private boolean turretActive;
  private boolean shooterActive;
  private IShooterFactory shooterFactory;
  private Shooter shooter;
  private ShootStraightCommand shootCommand;
  private ITurretFactory turretFactory;
  private Turret turret;
  private SetTurretAngleCommand turretAngleCommand;

  private Joystick joystick;
  private JoystickButton button5;
  private JoystickButton button6;
  public RegularRobotContainer() {
    shooterFactory = new DefaultShooterFactory();
    shooter = shooterFactory.makeShooter();
    shootCommand = new ShootStraightCommand(shooter, this);
    shooter.setDefaultCommand(shootCommand);
    turretFactory = new DefaultTurretFactory();
    turret = turretFactory.makeTurret();
    turretAngleCommand = new SetTurretAngleCommand(turret, this);
    turret.setDefaultCommand(turretAngleCommand);

    joystick = new Joystick(0);
    button5 = new JoystickButton(joystick, 5);
    button6 = new JoystickButton(joystick, 6);
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
    button5.whenPressed(new InstantCommand(() -> this.setShooterActive(true)));
    button5.whenReleased(new InstantCommand(() -> this.setShooterActive(false)));
    button6.whenPressed(turretAngleCommand);
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

  public void setShooterActive(boolean shooterActive){
    this.shooterActive = shooterActive;
  }

  public boolean getShooterActive(){
      return this.shooterActive;
  }

  public void setTurretActive(boolean turretActive){
    this.turretActive = turretActive;
  }

  public boolean getTurretActive(){
    return turretActive;
  }
}
