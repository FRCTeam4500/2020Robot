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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.IArmOI;
import frc.robot.subsystems.arm.factory.DefaultArmFactory;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.command.IndexBallsCommand;
import frc.robot.subsystems.indexer.factory.DefaultIndexerFactory;
import frc.robot.subsystems.indexer.factory.IIndexerFactory;
import frc.robot.subsystems.intake.IIntakeOI;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.command.ShootStraightCommand;
import frc.robot.subsystems.shooter.factory.DefaultShooterFactory;
import frc.robot.subsystems.shooter.factory.IShooterFactory;
import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.subsystems.swerve.normal.NormalSwerve;
import frc.robot.subsystems.turret.ITurretOI;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.DefaultTurretFactory;
import frc.robot.subsystems.turret.factory.ITurretFactory;
import frc.robot.subsystems.turret.command.ChangeTurretAngleCommand;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.command.TurretTrackingCommand;
import frc.robot.subsystems.vision.factory.DefaultVisionFactory;
import frc.robot.subsystems.vision.factory.IVisionFactory;
import frc.robot.subsystems.arm.command.ArmRunCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RegularRobotContainer implements ITurretOI, IShooterOI, IRobotContainer, ISwerveOI, IIntakeOI, IArmOI {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  private double turretAngle;
  private double shooterAngle;
  private boolean turretActive;
  private double turretDesiredAngle;
  private boolean intakeActive;
  private boolean armActive;
  private IShooterFactory shooterFactory;
  private Shooter shooter;
  private ShootStraightCommand shootCommand;
  private ITurretFactory turretFactory;
  private Turret turret;
  private ChangeTurretAngleCommand turretAngleCommand;
  private IVisionFactory visionFactory;
  private Vision vision;
  private TurretTrackingCommand turretTrackingCommand;
  private Arm arm;
  private IIndexerFactory indexerFactory;
  private Indexer indexer;
  private IndexBallsCommand indexerCommand;
  private DefaultArmFactory armFactory;
  private ArmRunCommand armCommand;
  private NormalSwerve swerve;


  private Joystick joystick;
  private JoystickButton button1;
  private JoystickButton button5;

  
  public RegularRobotContainer() {
    turretAngle = 0;
    shooterAngle = 0;
    turretActive = false;
    intakeActive = false;
    shooterFactory = new DefaultShooterFactory();
    shooter = shooterFactory.makeShooter();
    shootCommand = new ShootStraightCommand(shooter, this);
    shooter.setDefaultCommand(shootCommand);
    turretFactory = new DefaultTurretFactory();
    turret = turretFactory.makeTurret();
    turretAngleCommand = new ChangeTurretAngleCommand(turret, this);
    turret.setDefaultCommand(turretAngleCommand);
    visionFactory = new DefaultVisionFactory();
    vision = visionFactory.makeVision();
    turretTrackingCommand = new TurretTrackingCommand(vision, this);
    vision.setDefaultCommand(turretTrackingCommand);
    indexerFactory = new DefaultIndexerFactory();
    indexer = indexerFactory.makeIndexer();
    indexerCommand = new IndexBallsCommand(indexer, 0.5);
    indexer.setDefaultCommand(indexerCommand);


    swerve.setDefaultCommand(new RunCommand(() -> swerve.moveRobotCentric(joystick.getX(),joystick.getY(),joystick.getZ()),swerve));
    // Configure the button bindings
    joystick = new Joystick(0);
    button1 = new JoystickButton(joystick, 1);
    button5 = new JoystickButton(joystick, 5);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button5.whenPressed(new InstantCommand(() -> {intakeActive = true; armActive = true;}));
    button5.whenReleased(new InstantCommand(() -> {intakeActive = false; armActive = false;}));
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
  public double getTurretDesiredAngle() {
    return this.turretDesiredAngle;
  }

  public void setTurretDesiredAngle(double turretDesiredAngle) {
    this.turretDesiredAngle = turretDesiredAngle;
  }

  public double getX(){
    return joystick.getX();
  }

  public double getY(){
    return joystick.getY();
  }

  public double getZ(){
    return joystick.getZ();
  }

  public boolean getIntakeActive() {
    return intakeActive;
  }

  public boolean getArmActive(){
    return armActive;
  }
}
