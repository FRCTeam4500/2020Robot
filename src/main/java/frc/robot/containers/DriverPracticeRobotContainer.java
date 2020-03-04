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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_PreciseShootingCommand;
import frc.robot.autonomous.Autonomous_StartShootingCommand;
import frc.robot.autonomous.Autonomous_StopShootingCommand;
import frc.robot.autonomous.GenericAutonUtilities;
import frc.robot.autonomous.IPreciseShootingOI;
import frc.robot.autonomous.IndexBallsCommand;
import frc.robot.autonomous.VisionDistanceCalculator;
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

import static frc.robot.utility.ExtendedMath.withDeadzone;
import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;
import static frc.robot.autonomous.GenericAutonUtilities.createDefaultControllerBuilder;
/**
 * Add your docs here.
 */
public class DriverPracticeRobotContainer implements IRobotContainer{

    private SendableChooser<Command> autonomousChooser;
    
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
    
    private boolean useFancyIntakeCommand = true;
    private VisionDistanceCalculator visionDistanceCalculator;
    
    private 
    double 
    xSensitivity = 4,
    ySensitivity = 4,
    zSensitivity = 6,
    xDeadzone = 0.2,
    yDeadzone = 0.2,
    zDeadzone = 0.2;
    
    public DriverPracticeRobotContainer(){

        configureBasicOverrides();

        configureMainIntakeButton();

        configureShooter();
        
        resetGyroButton
        .whenPressed(() -> swerve.resetPose());

        configureSwerve();

        configureTurret();

        configureClimber();
        
        configureSmartDashboardControls();

        configureAutonomous();

        SmartDashboard.putData("Enable Servo", new InstantCommand(() -> climber.enableServo(),climber));
        SmartDashboard.putData("Disable Servo", new InstantCommand(() -> climber.disableServo(), climber));
    }
    private void configureAutonomous() {
        autonomousChooser = new SendableChooser<>();
        autonomousChooser.addOption(
            "Shoot and Cross The Line",
            createShootAndCrossTheLineCommand()
        ); 
        autonomousChooser.addOption(
            "Away From Center, Move Forward and Shoot", 
            createAwayFromCenterMoveForwardAndShootCommand()
        );
        autonomousChooser.addOption(
            "Away From Center, Move Backward and Shoot", 
            createAwayFromCenterMoveBackwardAndShootCommand()
        );
        autonomousChooser.addOption(
            "Citrus Compatible", 
            createCitrusCompatibleCommand());

        
        SmartDashboard.putData("Selected Auto", autonomousChooser);
    }
    private SequentialCommandGroup createCitrusCompatibleCommand() {
        return new InstantCommand(
            () -> swerve.resetPose(
                new Pose2d(13, -5.75, new Rotation2d(Math.PI))), 
                swerve)
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -800, -800))
      .andThen(new WaitCommand(2))
      .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
      .andThen(new WaitCommand(4))
      .andThen(
        new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder().withEndRotation(new Rotation2d(Math.PI + Math.PI * 1.2/7))
            .withTrajectory(tryGetDeployedTrajectory("CitrusCompatublePart1"))
            .buildController()
        )
      )
      .andThen(() -> swerve.moveFieldCentric(0, 0, 0),swerve)
      .andThen(() -> arm.setAngle(Math.PI/2),arm)
      .andThen(() -> intake.setSpeed(-1), intake)
      .andThen(() -> indexer.setSpeed(1), indexer)
      .andThen(
        //new IndexBallsCommand(indexer, intake, 1)
        new RunCommand(() -> swerve.moveFieldCentric(0.1, -0.25*2, 0), swerve)
        .withTimeout(2)
        .andThen(new RunCommand(() -> swerve.moveFieldCentric(0.1*2.4, -0.25*2.4, 0), swerve).withTimeout(0.7)
        .andThen(new RunCommand(() -> swerve.moveFieldCentric(0,0,1)).withTimeout(1))
        )
        )

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
            .buildController()
        )
      )
      .andThen(() -> swerve.moveFieldCentric(0, 0, 0), swerve)
      .andThen(
        new Autonomous_StartShootingCommand(indexer, shooter, -800, -800)
      )
      .andThen(new WaitCommand(2))
      .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
      .andThen(() -> indexer.setSpeed(-1), indexer)
      .andThen(() -> intake.setSpeed(1),intake)
      .andThen(new WaitCommand(4))
      .andThen(() -> indexer.setSpeed(0),indexer)
      .andThen(() -> intake.setSpeed(0),intake);
    }
    private SequentialCommandGroup createAwayFromCenterMoveBackwardAndShootCommand() {
        return new InstantCommand(
            () -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))),
            swerve).
        andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterBackward"))
            .withEndRotation(new Rotation2d(Math.PI))
            .buildController()))
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter));
    }
    private SequentialCommandGroup createAwayFromCenterMoveForwardAndShootCommand() {
        return new InstantCommand(
            () -> swerve.resetPose(new Pose2d(13, -2.5, new Rotation2d(Math.PI))),
            swerve)
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withTrajectory(tryGetDeployedTrajectory("AwayFromCenterForward"))
            .withEndRotation(new Rotation2d(Math.PI))
            .buildController()))
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter));
    }
    private SequentialCommandGroup createShootAndCrossTheLineCommand() {
        return new InstantCommand(
            () -> swerve.resetPose(
                new Pose2d(13, -5.75, new Rotation2d(Math.PI))), 
                swerve)
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        .andThen(new OdometricSwerve_AdvancedFollowTrajectoryCommand(
            swerve, 
            createDefaultControllerBuilder()
            .withEndRotation(new Rotation2d(Math.PI))
            .withTrajectory(tryGetDeployedTrajectory("CrossTheLine"))
            .buildController()
            )
        );
    }
    private void configureShooter() {
        visionDistanceCalculator = GenericAutonUtilities.makeEntropyVisionDistanceCalculator(vision);
        var shootCommand = new Autonomous_PreciseShootingCommand(shooter, indexer,
            // new frc.robot.autonomous.IPreciseShootingOI() {
            
            //     @Override
            //     public double getTopSpeed() {
            //         return SmartDashboard.getNumber("topSpeed", 0);
            //     }
            
            //     @Override
            //     public double getThreshold() {
            //         return SmartDashboard.getNumber("threshold", 0.0);
            //     }
            
            //     @Override
            //     public double getCoefficient() {
            //         return SmartDashboard.getNumber("coefficient", 1);
            //     }
            
            //     @Override
            //     public double getBottomSpeed() {
            //         return SmartDashboard.getNumber("bottomSpeed", 0.0);
            //     }
            // }
            new IPreciseShootingOI(){
            
                @Override
                public double getTopSpeed() {
                    double distance = Units.metersToFeet(visionDistanceCalculator.getDistanceFromTargetMeters());
                    return -12043 + 1244*distance - 60.7 *distance * distance + .905 * distance * distance * distance;
                }
            
                @Override
                public double getThreshold() {
                    return 500;
                }
            
                @Override
                public double getCoefficient() {
                    return 1;
                }
            
                @Override
                public double getBottomSpeed() {
                    double distance = Units.metersToFeet(visionDistanceCalculator.getDistanceFromTargetMeters());
                    return -7179 + 466*distance - 18.1*distance*distance + .211 * distance * distance * distance;
                }
            }
        );
        SmartDashboard.putNumber("topSpeed", -1000);
        SmartDashboard.putNumber("bottomSpeed", -1000);
        SmartDashboard.putNumber("coefficient", 1);
        SmartDashboard.putNumber("threshold", 45);

        shootButton
        .whileHeld(shootCommand);
    }
	private void configureSmartDashboardControls() {
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
        SmartDashboard.putData("Vision Distance Calculator", visionDistanceCalculator);
    }
    private void configureClimber() {
        climberUpButton
        .whenPressed(() -> {climber.setSpeed(1);climber.enableServo();}, climber)
        .whenReleased(() -> {climber.setSpeed(0); climber.disableServo();}, climber);

        climberDownButton
        .whenPressed(() -> {climber.setSpeed(-1); climber.enableServo();}, climber)
        .whenReleased(() -> {climber.setSpeed(0); climber.disableServo();}, climber);
    }
    private void configureTurret() {
        turret.setDefaultCommand(
            new PIDCommand(
                new PIDController(-6, 0, 0), 
                vision::getHorizontalOffset, 
                () -> 0, 
                turret::setTurretOutput, 
                turret, vision
            )
        );
    }
    private void configureSwerve() {
        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withDeadzone(-driveStick.getY(), yDeadzone)*ySensitivity, 
                withDeadzone(-driveStick.getX(), xDeadzone)*xSensitivity,
                withDeadzone(-driveStick.getZ() , zDeadzone)*zSensitivity
            );
        }, swerve));
    }
    private void configureMainIntakeButton() {
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
                indexer, 
                arm, intake
            ),
            () -> useFancyIntakeCommand
            )
        );
    }
    private void configureBasicOverrides() {
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
    }
    @Override
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
}
