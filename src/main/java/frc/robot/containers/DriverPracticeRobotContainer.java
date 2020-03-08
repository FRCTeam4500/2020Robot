/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
import static frc.robot.utility.ExtendedMath.withDeadzone;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.pshoot.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.IndexBallsCommand;
import frc.robot.autonomous.VisionDistanceCalculator;
import frc.robot.autonomous.pshoot.VisionPreciseShootingOI;
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
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.factory.HardwareTurretFactory;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Add your docs here.
 */
public class DriverPracticeRobotContainer implements IRobotContainer {

    private static final boolean coastingEnabled = false;

    private SendableChooser<Command> autonomousChooser;

    private Joystick driveStick = new Joystick(0), controlStick = new Joystick(1);
    private JoystickButton intakeInButton = new JoystickButton(controlStick, 5),
            intakeOutButton = new JoystickButton(controlStick, 3),
            indexerInButton = new JoystickButton(controlStick, 6),
            indexerOutButton = new JoystickButton(controlStick, 4),
            mainIntakeButton = new JoystickButton(controlStick, 1), backupIndexer = new JoystickButton(controlStick, 8),
            shootButton = new JoystickButton(controlStick, 2), climberUpButton = new JoystickButton(controlStick, 9),
            climberDownButton = new JoystickButton(controlStick, 11),
            resetGyroButton = new JoystickButton(driveStick, 5),
            alignWithLoadingBayButton = new JoystickButton(driveStick, 2);

    private Intake intake = new HardwareIntakeFactory().makeIntake();
    private Arm arm = new HardwareArmFactory().makeArm();
    private OdometricSwerve swerve = new EntropySwerveFactory().makeSwerve();
    private Indexer indexer = new HardwareIndexerFactory().makeIndexer();
    private Shooter shooter = new HardwareShooterFactory().makeShooter();
    private Turret turret = new HardwareTurretFactory().makeTurret();
    private VisionSubsystem limelight = new VisionSubsystem(new LimelightVisionComponent());

    private Climber climber = new HardwareClimberFactory().makeClimber();

    private boolean useFancyIntakeCommand = true;
    private VisionDistanceCalculator visionDistanceCalculator;
    private VisionPreciseShootingOI visionPreciseShootingOI;

    private double xSensitivity = 4, ySensitivity = 4, zSensitivity = 4, xDeadzone = 0.2, yDeadzone = 0.2,
            zDeadzone = 0.3;

    private double turretRadianOffset = 0.0;

    public DriverPracticeRobotContainer() {

        configureBasicOverrides();

        configureMainIntakeButton();

        configureShooter();

        resetGyroButton.whenPressed(() -> swerve.resetPose());
        swerve.resetPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));

        configureSwerve();

        configureTurret();

        configureClimber();

        configureSmartDashboardControls();

        configureAutonomous();

        backupIndexer.whileHeld(new IndexBallsCommand(indexer, intake, 1, 0));
    }

    private double getTurretRadianOffset() {
        return turretRadianOffset;
    }

    private void setTurretRadianOffset(double turretRadianOffset) {
        this.turretRadianOffset = turretRadianOffset;
    }

    private void configureAutonomous() {
        autonomousChooser = new SendableChooser<>();

        autonomousChooser.addOption(
            "Shoot and Cross The Line", 
            createShootAndCrossTheLineCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Forward and Shoot",
            createAwayFromCenterMoveForwardAndShootCommand());

        autonomousChooser.addOption(
            "Away From Center, Move Backward and Shoot",
            createAwayFromCenterMoveBackwardAndShootCommand());

        autonomousChooser.addOption(
            "Citrus Compatible", 
            createCitrusCompatibleCommand());

        autonomousChooser.addOption(
            "Trench Citrus Compatible Primary", 
            createTrenchCitrusCompatiblePartACommand());

        autonomousChooser.addOption(
            "Trench Citrus Compatible Second", 
            createTrenchCitrusCompatibleBCommand());

        SmartDashboard.putData("Selected Auto", autonomousChooser);
    }

    private SequentialCommandGroup createTrenchCitrusCompatibleBCommand() {
        return createTrenchCitrusPart1Command()
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2B"))
                    .withEndRotation(new Rotation2d(7 * Math.PI / 6))
                    .buildController()))
            .andThen(() -> arm.setAngle(Math.PI / 2), arm)
            .andThen(new IndexBallsCommand(indexer, intake, 1, 0.9).withTimeout(5))
            .andThen(() -> arm.setAngle(0), arm)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart3B"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
                .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                        .withTimeout(4));
    }

    private SequentialCommandGroup createTrenchCitrusCompatiblePartACommand() {
        return createTrenchCitrusPart1Command()
        .andThen(createTrenchCitrusPart1Command())
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve,
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart2A"))
            .buildController()))
        .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI).withTimeout(3));
    }

    private CommandGroupBase createTrenchCitrusPart1Command() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve)
            .andThen(() -> arm.setAngle(Math.PI / 2), arm)
            .andThen(
                new IndexBallsCommand(indexer, intake, 1, 0.9).withTimeout(4)
                .alongWith(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withTrajectory(tryGetDeployedTrajectory("TrenchCitrusCompatiblePart1"))
                    .buildController())))
            .andThen(() -> arm.setAngle(0), arm);
    }

    private SequentialCommandGroup createCitrusCompatibleCommand() {
        return new InstantCommand(() -> turretRadianOffset = Units.degreesToRadians(3.5))
            .andThen(new InstantCommand(() -> swerve.resetPose(new Pose2d(12.565, -4.875, new Rotation2d(Math.PI))), swerve))
            .andThen(new Autonomous_PreciseShootingCommand(shooter, indexer, -3390, -2520, 1.47, 500).withTimeout(3))
            .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                swerve,
                createDefaultControllerBuilder()
                .withEndRotation(new Rotation2d(Math.PI + Math.PI * 1.2 / 7))
                .withTrajectory(tryGetDeployedTrajectory("CitrusCompatublePart1"))
                .withMaxVelocity(4.0)
                .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve).andThen(() -> arm.setAngle(Math.PI / 2.1), arm)
            .andThen(
                new IndexBallsCommand(indexer, intake, 1.0, 0.9)
                .raceWith(
                    new RunCommand(() -> swerve.moveFieldCentric(0.1, -0.25 * 2, 0), swerve)
                    .withTimeout(2)
                    .andThen(
                        new RunCommand(() -> swerve.moveFieldCentric(0.1 * 2.2, -0.25 * 2.2, 0), swerve)
                        .withTimeout(0.7)
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, 1))
                            .withTimeout(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(-0.1, 0, 0))
                            .withTimeout(1))
                        .andThen(new WaitCommand(1))
                        .andThen(
                            new RunCommand(() -> swerve.moveFieldCentric(0, 0, -2))
                            .withTimeout(1)))))
            .andThen(() -> turretRadianOffset = 0.0)
            .andThen(() -> arm.setAngle(0), arm)
            .andThen(() -> intake.setSpeed(0), intake)
            .andThen(() -> indexer.setSpeed(0), indexer)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withInitialAllowableTranslationError(0.5)
                    .withFinalAllowableTranslationError(0.02)
                    .withTrajectory(tryGetDeployedTrajectory("CitrusCompatiblePart3"))
                    .withMaxVelocity(4.0)
                    .buildController())
                .withTimeout(0))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(4));
    }

    private SequentialCommandGroup createAwayFromCenterMoveBackwardAndShootCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterBackward"))
                    .withEndRotation(new Rotation2d(Math.PI)).buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(2));
    }

    private SequentialCommandGroup createAwayFromCenterMoveForwardAndShootCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterForward"))
                    .withEndRotation(new Rotation2d(Math.PI))
                    .buildController()))
            .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI)
                .withTimeout(2));
    }

    private SequentialCommandGroup createShootAndCrossTheLineCommand() {
        return new InstantCommand(() -> swerve.resetPose(new Pose2d(13, -5.75, new Rotation2d(Math.PI))), swerve)
            .andThen(
                new Autonomous_PreciseShootingCommand(shooter, indexer, -3390, -2520, 1.47, 500)
                .withTimeout(4))
            .andThen(
                new OdometricSwerve_AdvancedFollowTrajectoryCommand(
                    swerve,
                    createDefaultControllerBuilder()
                    .withEndRotation(new Rotation2d(Math.PI))
                    .withTrajectory(tryGetDeployedTrajectory("CrossTheLine"))
                    .buildController()));
    }

    private void configureShooter() {
        visionDistanceCalculator = GenericAutonUtilities.makeEntropyVisionDistanceCalculator(limelight);
        visionPreciseShootingOI = new VisionPreciseShootingOI(visionDistanceCalculator);
        var shootCommand = new Autonomous_PreciseShootingCommand(shooter, indexer, visionPreciseShootingOI);

        shootButton.whileHeld(shootCommand);
    }

    private void configureSmartDashboardControls() {
        SmartDashboard.putData("Control Preferences", new Sendable() {

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
        SmartDashboard.putData("Vision Distance Calculator", visionDistanceCalculator);

        SmartDashboard.putData("Indexer", indexer);

        SmartDashboard.putData(new Sendable(){
        
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Turret Radian Offset", () -> getTurretRadianOffset(), value -> setTurretRadianOffset(value));
            }
        });

    }

    private void configureClimber() {
        climberUpButton.whenPressed(() -> climber.setSpeed(1), climber).whenReleased(() -> climber.setSpeed(0),
                climber);

        climberDownButton.whenPressed(() -> climber.setSpeed(-1), climber).whenReleased(() -> climber.setSpeed(0),
                climber);
    }

    private void configureTurret() {
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-3, 0, 0), 
                limelight::getHorizontalOffset,
                this::getTurretRadianOffset, 
                turret::setTurretOutput, 
                turret,limelight));
    }

    private void configureSwerve() {
        swerve.setDefaultCommand(new RunCommand(() -> {
            var forwardSpeed = withDeadzone(-driveStick.getY(), yDeadzone) * ySensitivity;
            var leftwardSpeed = withDeadzone(-driveStick.getX(), xDeadzone) * xSensitivity;
            var counterClockwardSpeed = withDeadzone(-driveStick.getZ(), zDeadzone) * zSensitivity;

            if (forwardSpeed == 0 && leftwardSpeed == 0 && counterClockwardSpeed == 0 && coastingEnabled) {
                swerve.coast();
            } else {
                swerve.moveFieldCentric(forwardSpeed, leftwardSpeed, counterClockwardSpeed);
            }
        }, swerve));
    }

    private void configureMainIntakeButton() {
        mainIntakeButton
        .whileHeld(
            new ConditionalCommand(
                new IndexBallsCommand(indexer, intake, 1, 0.9)
                .alongWith(
                    new FunctionalCommand(
                        () -> arm.setAngle(Math.PI / 2), 
                        () -> {}, 
                        interrupted -> arm.setAngle(0), 
                        () -> false, 
                        arm)), 
                    new FunctionalCommand(
                        () -> {
                            indexer.setSpeed(1);
                            intake.setSpeed(-1);
                            arm.setAngle(Math.PI / 2);
                        }, 
                        () -> {}, 
                        interrupted -> {
                            indexer.setSpeed(0);
                            intake.setSpeed(0);
                            arm.setAngle(0);
                        }, 
                        () -> false, 
                        indexer, arm, intake), 
                () -> useFancyIntakeCommand));
    }

    private void configureBasicOverrides() {
        intakeInButton
        .whenPressed(() -> {
            intake.setSpeed(1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        intakeOutButton
        .whenPressed(() -> {
            intake.setSpeed(-1);
            arm.setAngle(Math.PI);
        }, intake, arm)
        .whenReleased(() -> {
            intake.setSpeed(0);
            arm.setAngle(0);
        }, intake, arm);

        indexerInButton
        .whenPressed(() -> indexer.setSpeed(1), indexer)
        .whenReleased(() -> indexer.setSpeed(0),indexer);

        indexerOutButton
        .whenPressed(() -> indexer.setSpeed(-1), indexer)
        .whenReleased(() -> indexer.setSpeed(0), indexer);
    }

    @Override
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
