/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.IndexBallsCommand;
import frc.robot.components.hardware.LimelightVisionComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.factory.HardwareIntakeFactory;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.HardwareArmFactory;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.factory.HardwareClimberFactory;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.factory.HardwareIndexerFactory;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.factory.HardwareShooterFactory;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;

import static frc.robot.utility.ExtendedMath.withDeadzone;
/**
 * Add your docs here.
 */
public class DriverPracticeRobotContainer implements IRobotContainer{
    private Joystick driveStick = new Joystick(0), controlStick = new Joystick(1);
    private JoystickButton 
    intakeInButton = new JoystickButton(controlStick, 5),
    intakeOutButton = new JoystickButton(controlStick, 3),
    indexerInButton  = new JoystickButton(controlStick,6),
    indexerOutButton = new JoystickButton(controlStick, 4),
    mainIntakeButton = new JoystickButton(controlStick, 1),
    shootButton = new JoystickButton(controlStick,2),
    climberUpButton = new JoystickButton(controlStick, 9),
    climberDownButton = new JoystickButton(controlStick, 11),
    resetGyroButton = new JoystickButton(driveStick, 5)
    ;

    private Intake intake = new HardwareIntakeFactory().makeIntake();
    private Arm arm = new HardwareArmFactory().makeArm();
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    private Indexer indexer = new HardwareIndexerFactory().makeIndexer();
    private Shooter shooter = new HardwareShooterFactory().makeShooter();
    private Turret turret = new HardwareTurretFactory().makeTurret();
    private VisionSubsystem vision = new VisionSubsystem(new LimelightVisionComponent());
    private Climber climber = new HardwareClimberFactory().makeClimber();

    private boolean useFancyIntakeCommand = false;

    private 
    double 
    xSensitivity = 1,
    ySensitivity = 1,
    zSensitivity = 1,
    xDeadzone = 0.2,
    yDeadzone = 0.2,
    zDeadzone = 0.2;
    
    public DriverPracticeRobotContainer(){

        intakeInButton
        .whenPressed(() -> intake.setSpeed(-1), intake)
        .whenReleased(() -> intake.setSpeed(0), intake);

        intakeOutButton
        .whenPressed(() -> intake.setSpeed(1), intake)
        .whenReleased(() -> intake.setSpeed(0));

        indexerInButton
        .whenPressed(() -> indexer.setSpeed(1), indexer)
        .whenReleased(() -> indexer.setSpeed(0), indexer);

        indexerOutButton
        .whenPressed(() -> indexer.setSpeed(-1), indexer)
        .whenReleased(() -> indexer.setSpeed(0), indexer);

        mainIntakeButton
        .whileHeld(new ConditionalCommand(
            new IndexBallsCommand(indexer, intake, 1)
            .alongWith(
                new FunctionalCommand(
                    () -> arm.setAngle(Math.PI/2), 
                    () -> {}, 
                    interrupted -> arm.setAngle(0), 
                    () -> false, 
                    arm)
                ),
            new FunctionalCommand(
                () ->{                 
                    indexer.setSpeed(1);
                    intake.setSpeed(-1);
                    arm.setAngle(Math.PI/2);
                }, 
                () -> {}, 
                interrupted -> {
                    indexer.setSpeed(0);
                    intake.setSpeed(0);
                    arm.setAngle(0);
                }, 
                () -> false,
                indexer, arm, intake
            ),
            () -> useFancyIntakeCommand
            )
        );

        if(useFancyIntakeCommand){
            mainIntakeButton
            .whileHeld(
                new IndexBallsCommand(indexer, intake, 1)
                .alongWith(new InstantCommand(() -> arm.setAngle(Math.PI/2), arm)))
            .whenReleased(() -> arm.setAngle(0), arm);
        }else{
            mainIntakeButton
            .whenPressed(() -> {
                indexer.setSpeed(1);
                intake.setSpeed(-1);
                arm.setAngle(Math.PI/2);
            }, indexer, intake, arm)
            .whenReleased(() -> {
                indexer.setSpeed(0);
                intake.setSpeed(0);
                arm.setAngle(0);
            }, indexer, intake, arm);
        }

        var shootCommand = new Autonomous_PreciseShootingCommand(shooter, indexer);
        shootCommand.createSmartDashboardEntries();

        shootButton
        .whileHeld(shootCommand);
        
        resetGyroButton
        .whenPressed(() -> swerve.resetPose());

        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withDeadzone(-driveStick.getY(), yDeadzone)*ySensitivity, 
                withDeadzone(-driveStick.getX(), xDeadzone)*xSensitivity,
                withDeadzone(-driveStick.getZ() , zDeadzone)*zSensitivity
            );
        }, swerve));

        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-6, 0, 0), 
                vision::getHorizontalOffset, 
                () -> 0, 
                turret::setTurretOutput, 
                turret, vision
            )
        );

        climberUpButton
        .whenPressed(() -> {climber.setSpeed(-1); climber.disableServo();}, climber)
        .whenReleased(() -> {climber.setSpeed(0); climber.enableServo();}, climber);

        climberDownButton
        .whenPressed(() -> {climber.setSpeed(1); climber.disableServo();}, climber)
        .whenReleased(() -> {climber.setSpeed(0); climber.enableServo();}, climber);
        


        SmartDashboard.putData("Control Preferences", new Sendable(){
        
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Use Sensors When Indexing", () -> useFancyIntakeCommand, value -> useFancyIntakeCommand = value);
                builder.addDoubleProperty("X Axis Sensitivity", () -> xSensitivity, value -> xSensitivity = value);
                builder.addDoubleProperty("Y Axis Sensitivity", () -> ySensitivity, value -> ySensitivity = value);
                builder.addDoubleProperty("Z Axis Sensitivity", () -> zSensitivity, value -> zSensitivity = value);
                builder.addDoubleProperty("X Axis Deadzone", () -> xDeadzone, value -> xDeadzone = value);
                builder.addDoubleProperty("Y Axis Deadzone", () -> yDeadzone, value -> yDeadzone = value);
                builder.addDoubleProperty("Z Axis Deadzone", () -> zDeadzone, value -> zDeadzone = value);
            }
        });

        SmartDashboard.putData("Swerve Transform", new OdometricSwerveDashboardUtility(swerve));
    }
}
