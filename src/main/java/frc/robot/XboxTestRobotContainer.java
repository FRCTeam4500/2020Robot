/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_FollowTrajectoryCommand;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_ResetPoseCommand;
import frc.robot.subsystems.swerve.odometric.factory.OdometricSimulatedSwerveFactory;

/**
 * Add your docs here.
 */
public class XboxTestRobotContainer implements IRobotContainer{
    private XboxController controller = new XboxController(0);
    private OdometricSimulatedSwerveFactory factory = new OdometricSimulatedSwerveFactory();
    private OdometricSwerve swerve = factory.makeSwerve();
    private OdometricSwerveDashboardUtility utility = new OdometricSwerveDashboardUtility(swerve);
    public XboxTestRobotContainer(){
        swerve.setDefaultCommand(
            new RunCommand(() -> 
                swerve.moveRobotCentric(
                   withDeadzone(controller.getY(Hand.kLeft),0.2)*10,
                    withDeadzone(controller.getX(Hand.kLeft),0.2)*10, 
                    withDeadzone(controller.getX(Hand.kRight),0.2)*3
                ), 
                swerve
            )
        );
        SendableRegistry.addLW(utility, "Swerve", "Utility");
        SmartDashboard.putData(new OdometricSwerve_ResetPoseCommand(new Pose2d(),swerve));
        var firstCommand = factory.makeMoveToPoseCommand("Translational Auto",swerve, new Pose2d());
        firstCommand.getCounterClockwardController().setTolerance(Double.POSITIVE_INFINITY);
        firstCommand.getCounterClockwardController().setPID(0, 0, 0);
        var secondCommand = factory.makeMoveToPoseCommand("Rotational Auto", swerve, new Pose2d());
        SmartDashboard.putData(firstCommand.andThen(secondCommand));
        SmartDashboard.putData(makeTrajectoryCommand());

        
    }
    private double withDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone){
            return 0;
        }else{
            return value;
        }
    }
    private OdometricSwerve_FollowTrajectoryCommand makeTrajectoryCommand(){
        var config = new TrajectoryConfig(2, 1);
        var trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(), 
            List.of(new Translation2d(2,2), new Translation2d(4,-2)),
            new Pose2d(6, 0, new Rotation2d()), 
            config);

        trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(), new Pose2d(5,0,new Rotation2d()), config);
        return new OdometricSwerve_FollowTrajectoryCommand(swerve, trajectory, new RamseteController());
    }
}
