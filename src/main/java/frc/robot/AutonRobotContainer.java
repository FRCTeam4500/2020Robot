/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.Autonomous_StartShootingCommand;
import frc.robot.autonomous.Autonomous_StopShootingCommand;
import frc.robot.autonomous.ExtendedTrajectoryUtilities;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;
import frc.robot.subsystems.indexer.command.IndexBallsCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.factory.EntropySwerveFactory;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_FollowTrajecoryCommand;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_ResetPoseCommand;

import static frc.robot.utility.ExtendedMath.withDeadzone;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * Add your docs here.
 */
public class AutonRobotContainer implements IRobotContainer{
    private OdometricSwerve swerve;
    private Joystick joystick;
    private JoystickButton resetGyro;
    private Intake intake;
    private Shooter shooter;
    private Arm arm;
    private Indexer indexer;
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
        resetGyro.whenPressed(() -> swerve.resetGyro(),swerve);

        swerve.setDefaultCommand(new RunCommand(() -> {
            swerve.moveFieldCentric(
                withDeadzone(-joystick.getY()*2, 0.3*2),
                withDeadzone(-joystick.getX()*2, 0.3*2),
                withDeadzone(joystick.getZ()*2, 0.3*2)
            );
        }
        , swerve));

        SmartDashboard.putData("Swerve Positions",new OdometricSwerveDashboardUtility(swerve));

        var resetPose = new OdometricSwerve_ResetPoseCommand(new Pose2d(13, -5.75, new Rotation2d()), swerve);
        resetPose.schedule();

        SmartDashboard.putData("Reset Pose",resetPose);

        SmartDashboard.putData("Stop Shooting", new Autonomous_StopShootingCommand(indexer, shooter));
        SmartDashboard.putData("Start Shooting", new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000));

        addShootAndCrossTheLineCommand();
        addCitrusCompatabileCommand();
        addAwayFromCenterCommand();
        addAwayFromCenterBackwardCommand();
        addCitrusCompatibleAndShootAgainCommand();


        addAutonCommand("CrossTheLine");
        addAutonCommand("CitrusCompatabile");
        addAutonCommand("AwayFromCenterForward");
        

    }
    private CommandBase makeMoveToTranslationCommand(String trajectoryName) {
        var pid = new PIDController(3, 0, 0);
        pid.setTolerance(0.1);
        return new OdometricSwerve_FollowTrajecoryCommand(swerve, pid, ExtendedTrajectoryUtilities.tryGetDeployedTrajectory(trajectoryName));
    }
    private void addAutonCommand(String trajectoryName){
        SmartDashboard.putData("Run "+trajectoryName, makeMoveToTranslationCommand(trajectoryName));
    }
    private void addShootAndCrossTheLineCommand(){
        SmartDashboard.putData("Auton Shoot and Cross The Line", 
        new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000)
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        .andThen(makeMoveToTranslationCommand("CrossTheLine"))
        );
    }
    private void addCitrusCompatabileCommand(){
        SmartDashboard.putData("Auton Citrus Compatabile",
        new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000)
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        .andThen(makeMoveToTranslationCommand("CitrusCompatabile"))
        );
        
    }
    private void addAwayFromCenterCommand(){
        SmartDashboard.putData("Auton Away From Center Forward",
        makeMoveToTranslationCommand("AwayFromCenterForward")
        .andThen(new Autonomous_StartShootingCommand(indexer, shooter, -3000, -3000))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(new WaitCommand(2))
        .andThen(new Autonomous_StopShootingCommand(indexer, shooter))
        );
    }
    private void addAwayFromCenterBackwardCommand(){
        SmartDashboard.putData("Auton Away From Center Backward",
        makeMoveToTranslationCommand("AwayFromCenterBackward")
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
        .andThen(makeMoveToTranslationCommand("CitrusCompatabile"))
        .andThen(() -> swerve.moveFieldCentric(0, 0, 0))
        .andThen(() -> arm.setAngle(Math.PI/2),arm)
        .andThen(new IndexBallsCommand(indexer, intake, 1)).withTimeout(3)
        .andThen(new WaitCommand(3))
        .andThen(() -> arm.setAngle(0.0),arm)
        .andThen(makeMoveToTranslationCommand("CitrusCompatibleComeBackPlease")));
    }
}
