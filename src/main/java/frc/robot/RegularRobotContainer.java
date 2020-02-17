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
import frc.robot.subsystems.intake.IIntakeOI;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.command.IntakeRunCommand;
import frc.robot.subsystems.intake.factory.DefaultIntakeFactory;
import frc.robot.subsystems.intake.factory.IIntakeFactory;
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
public class RegularRobotContainer implements ITurretOI, IShooterOI, IRobotContainer, IIntakeOI {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  private double turretAngle;
  private double shooterAngle;
  private boolean turretActive;
  private boolean intakeActive;
  private IShooterFactory shooterFactory;
  private Shooter shooter;
  private ShootStraightCommand shootCommand;
  private ITurretFactory turretFactory;
  private Turret turret;
  private SetTurretAngleCommand turretAngleCommand;
  private IIntakeFactory intakeFactory;
  private Intake intake;
  private IntakeRunCommand intakeCommand;

  private Joystick joystick;
  private JoystickButton button1;

  public RegularRobotContainer() {
    shooterFactory = new DefaultShooterFactory();
    shooter = shooterFactory.makeShooter();
    shootCommand = new ShootStraightCommand(shooter, this);
    shooter.setDefaultCommand(shootCommand);
    turretFactory = new DefaultTurretFactory();
    turret = turretFactory.makeTurret();
    turretAngleCommand = new SetTurretAngleCommand(turret, this);
    turret.setDefaultCommand(turretAngleCommand);
    intakeFactory = new DefaultIntakeFactory();
    intake = intakeFactory.makeIntake();
    intakeCommand = new IntakeRunCommand(intake, this);
    intake.setDefaultCommand(intakeCommand);

    joystick = new Joystick(0);
    button1 = new JoystickButton(joystick, 1);
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
    button1.whenPressed(new InstantCommand(() -> this.setIntakeActive(true)));
    button1.whenReleased(new InstantCommand(() -> this.setIntakeActive(false)));
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

  public boolean getShooterActive(){
      return this.turretActive;
  }

  public void setIntakeActive(boolean intakeActive){
    this.intakeActive = intakeActive;
  }
  public boolean getIntakeActive(){
    return intakeActive;
  }
}