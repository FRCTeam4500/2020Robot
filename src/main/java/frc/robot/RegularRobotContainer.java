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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.command.IntakeRunCommand;
import frc.robot.subsystems.intake.factory.DefaultIntakeFactory;
import frc.robot.subsystems.shooter.IShooterOI;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.command.DefaultShootCommand;
import frc.robot.subsystems.shooter.factory.DefaultShooterFactory;
import frc.robot.subsystems.shooter.factory.IShooterFactory;
import frc.robot.subsystems.swerve.ISwerve;
import frc.robot.subsystems.swerve.ISwerveFactory;
import frc.robot.subsystems.swerve.ISwerveOI;
import frc.robot.subsystems.swerve.normal.command.MoveFieldCentricCommand;
import frc.robot.subsystems.swerve.normal.factory.NormalSwerveFactory;
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
public class RegularRobotContainer implements IRobotContainer {

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  private double turretDesiredAngle;
  private IShooterFactory shooterFactory;
  private Shooter shooter;
  private DefaultShootCommand shootCommand;
  private ITurretFactory turretFactory;
  private Turret turret;
  private ChangeTurretAngleCommand turretAngleCommand;
  private IVisionFactory visionFactory;
  private Vision vision;
  private Arm arm;
  private IIndexerFactory indexerFactory;
  private Indexer indexer;
  private IndexBallsCommand indexerCommand;
  private DefaultArmFactory armFactory;
  private ArmRunCommand armCommand;
  private ISwerveFactory swerveFactory;
  private ISwerve swerve;
  private DefaultIntakeFactory intakeFactory;
  private Intake intake;
  private IntakeRunCommand intakeCommand;



  private Joystick joystick;
  private JoystickButton button1;
  private JoystickButton button2;
  private JoystickButton button5;
  private JoystickButton button9;

  
  public RegularRobotContainer() {
    joystick = new Joystick(0);
    button1 = new JoystickButton(joystick, 1);
    button2 = new JoystickButton(joystick,2);
    button5 = new JoystickButton(joystick, 5);
    button9 = new JoystickButton(joystick,9);


    shooterFactory = new DefaultShooterFactory();
    shooter = shooterFactory.makeShooter();
    shootCommand = new DefaultShootCommand(shooter, button1::get);
    shooter.setDefaultCommand(shootCommand);
    visionFactory = new DefaultVisionFactory();
    vision = visionFactory.makeVision();
    turretFactory = new DefaultTurretFactory();
    turret = turretFactory.makeTurret();
    turretAngleCommand = new ChangeTurretAngleCommand(turret, vision::getHorizontalOffset);
    turret.setDefaultCommand(turretAngleCommand);
    indexerFactory = new DefaultIndexerFactory();
    indexer = indexerFactory.makeIndexer();
    indexerCommand = new IndexBallsCommand(indexer, 0.5);
    indexer.setDefaultCommand(indexerCommand);
    swerveFactory = new NormalSwerveFactory();
    swerve = swerveFactory.makeSwerve();
    swerve.setDefaultCommand(new RunCommand(() -> swerve.moveFieldCentric(withDeadzone(joystick.getX()),
            withDeadzone(joystick.getY()),withDeadzone(joystick.getZ()))));
    armFactory = new DefaultArmFactory();
    arm = armFactory.makeArm();
    armCommand = new ArmRunCommand(arm, button2::get);
    arm.setDefaultCommand(armCommand);
    intakeFactory = new DefaultIntakeFactory();
    intake = intakeFactory.makeIntake();
    intakeCommand = new IntakeRunCommand(intake, button2::get);
    intake.setDefaultCommand(intakeCommand);


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
    button1.whenPressed(() -> {indexer.setSpeed(1);});
    button1.whenReleased(() -> {indexer.setSpeed(0);});
    button9.whenPressed(() -> swerve.resetGyro());
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



  public double getTurretDesiredAngle() {
    return this.turretDesiredAngle;
  }

  public void setTurretDesiredAngle(double turretDesiredAngle) {
    this.turretDesiredAngle = turretDesiredAngle;
  }




  public double withDeadzone(double number){
    if (Math.abs(number) <= 0.2){
      return 0;
    }
    else{
      return number;
    }
  }

  public void onDisable(){
    swerve.moveFieldCentric(0,0,0);
  }
}
