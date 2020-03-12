/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.containers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_StartShootingCommand;
import frc.robot.autonomous.Autonomous_StopShootingCommand;
import frc.robot.autonomous.Autonomous_IndexBallsCommand;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveController;
import frc.robot.subsystems.swerve.odometric.command.AdvancedSwerveControllerBuilder;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_AdvancedFollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_FollowTrajecoryCommand;

import static frc.robot.utility.ExtendedMath.withHardDeadzone;
import static frc.robot.autonomous.ExtendedTrajectoryUtilities.tryGetDeployedTrajectory;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * Add your docs here.
 */
public class AutonRobotContainer implements IRobotContainer{
    private OdometricSwerve swerve;
    private Joystick joystick;
    private JoystickButton resetGyro, indexCommand;
    private Intake intake;
    private Shooter shooter;
    private Arm arm;
    private Indexer indexer;
    private CommandBase citrusCompatible;
    public AutonRobotContainer(){
        swerve = new EntropySwerveFactory().makeSwerve();

        arm = new Arm(new TalonSRXComponent(8));
        intake = new Intake(new TalonSRXComponent(5));
        shooter = new Shooter(
            new SparkMaxComponent(14, MotorType.kBrushless), 
        new SparkMaxComponent(13, MotorType.kBrushless));
        indexer = new Indexer(
            new TalonSRXComponent(12), 
            new NetworkTableBallSensor("Sensor0", 40), 
            new NetworkTableBallSensor("Sensor1", 40), 
            new NetworkTableBallSensor("Sensor2", 40), 
            new NetworkTableBallSensor("Sensor3", 40), 
            new NetworkTableBallSensor("Sensor4", 40), 
            new NetworkTableBallSensor("Sensor5", 40)
        );

        joystick = new Joystick(0);
        
        resetGyro = new JoystickButton(joystick, 1);
        resetGyro.whenPressed(() -> swerve.resetPose(swerve.getCurrentPose().getTranslation()),swerve);

        indexCommand = new JoystickButton(joystick, 2);
        indexCommand.whenHeld(new Autonomous_IndexBallsCommand(indexer, intake, 1,0.9));
        
        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withHardDeadzone(-joystick.getY()*2, 0.3*2),
                withHardDeadzone(-joystick.getX()*2, 0.3*2),
                withHardDeadzone(-joystick.getZ()*2, 0.3*2)
            );
        }
        , swerve));

        SmartDashboard.putData("Swerve Positions",new OdometricSwerveDashboardUtility(swerve));

        var basePose = new Pose2d(new Translation2d(13,-5.75),new Rotation2d(Math.PI));
        var resetPose = new InstantCommand(() -> swerve.resetPose(basePose),swerve);
        swerve.resetPose(basePose);

        SmartDashboard.putData("Reset Pose",resetPose);

        SmartDashboard.putData("Stop Shooting", new Autonomous_StopShootingCommand(indexer, shooter));
        SmartDashboard.putData("Start Shooting", new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000));
        SmartDashboard.putData("Stop Intake", new InstantCommand(() -> intake.setSpeed(0),intake));


        SmartDashboard.putData("Outtake Ball", new InstantCommand(() -> intake.setSpeed(1),intake));

        SmartDashboard.putData("Zero Arm", new InstantCommand(() -> arm.setAngle(0),arm));
        addAutonCommand("CrossTheLine", 
        createDefaultControllerBuilder()
        );
        addAutonCommand("CitrusCompatublePart1",
        createDefaultControllerBuilder().withEndRotation(new Rotation2d(Math.PI + Math.PI/6))
        );
        addAutonCommand("AwayFromCenterForward",
        createDefaultControllerBuilder().withEndRotation(new Rotation2d(Math.PI))
        );
        addAutonCommand("AwayFromCenterBackward", createDefaultControllerBuilder().withEndRotation(new Rotation2d(Math.PI)));

        addCitrusCompatabileCommand();
        

    }
    private AdvancedSwerveControllerBuilder createDefaultControllerBuilder(){
        return         new AdvancedSwerveControllerBuilder()
        .withInitialAllowableTranslationError(0.1)
        .withFinalAllowableTranslationError(0.1)
        .withAllowableRotationError(0.1)
        .withTranslationsEnabled(true)
        .with_kP(3)
        .with_kW(3)
        .withRotationsEnabled(true)
        .withEndRotation(new Rotation2d())
        .withMaxVelocity(2.4);
    }
    private CommandBase makeMoveToTranslationCommand(String trajectoryName) {
        var pid = new PIDController(3, 0, 0);
        pid.setTolerance(0.1);
        return new OdometricSwerve_FollowTrajecoryCommand(swerve, pid, tryGetDeployedTrajectory(trajectoryName));
    }
    private CommandBase makeAdvancedMoveToTranslationCommand(String trajectoryName){
        var controller = new AdvancedSwerveController(0.1, 0.1, false, 0.1, true, 3, 0, new Rotation2d(),2.4,tryGetDeployedTrajectory(trajectoryName).getStates().toArray(Trajectory.State[]::new));
        return new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, controller);
    }
    private void addAutonCommand(String trajectoryName){
        SmartDashboard.putData("Run Path "+trajectoryName, makeAdvancedMoveToTranslationCommand(trajectoryName));
    }
    private void addAutonCommand(String trajectoryName, AdvancedSwerveControllerBuilder builder){
        SmartDashboard.putData("Run Path "+trajectoryName, new OdometricSwerve_AdvancedFollowTrajectoryCommand(swerve, builder.withTrajectory(tryGetDeployedTrajectory(trajectoryName)).buildController()));
    }
    private void addShootAndCrossTheLineCommand(){
        SmartDashboard.putData("Auton Shoot and Cross The Line", 
        new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000)
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        .andThen(makeAdvancedMoveToTranslationCommand("CrossTheLine"))
        );
    }
    private void addCitrusCompatabileCommand(){
        citrusCompatible = new Autonomous_StartShootingCommand(indexer, shooter, -800, -800)
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

        
        
        SmartDashboard.putData("Auton Citrus Compatible", citrusCompatible);
        
        
    }
    private void addAwayFromCenterCommand(){
        SmartDashboard.putData("Auton Away From Center Forward",
        makeAdvancedMoveToTranslationCommand("AwayFromCenterForward")
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        );
    }
    private void addAwayFromCenterBackwardCommand(){
        SmartDashboard.putData("Auton Away From Center Backward",
        makeAdvancedMoveToTranslationCommand("AwayFromCenterBackward")
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        );
    }
    private void addCitrusCompatibleAndShootAgainCommand(){
        SmartDashboard.putData("Auton Citrus Compatible With Additional Shooting",
        new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000)
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        .andThen(new WaitCommand(3))
        .andThen(makeAdvancedMoveToTranslationCommand("CitrusCompatabile"))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(() -> arm.setAngle(Math.PI/2),arm)
        .andThen(new Autonomous_IndexBallsCommand(indexer, intake, 1,0.9)).withTimeout(3)
        .andThen(new WaitCommand(3))
        .andThen(() -> arm.setAngle(0.0),arm)
        .andThen(makeAdvancedMoveToTranslationCommand("CitrusCompatibleComeBackPlease")));
    }
}
