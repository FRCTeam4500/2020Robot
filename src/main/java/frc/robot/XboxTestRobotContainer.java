/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.odometric.OdometricSwerve;
import frc.robot.subsystems.swerve.odometric.OdometricSwerveDashboardUtility;
import frc.robot.subsystems.swerve.odometric.command.OdometricSwerve_MoveToTranslationCommand;
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
        SmartDashboard.putData("Reset Pose",new OdometricSwerve_ResetPoseCommand(new Pose2d(),swerve));
        SmartDashboard.putData("Run Auto",makeMoveToTranslationCommand());


        
    }
    private double withDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone){
            return 0;
        }else{
            return value;
        }
    }
    private CommandBase makeMoveToTranslationCommand(){
        return makeMoveToTranslationCommand(0, -2, 0)
        .andThen(makeMoveToTranslationCommand(1,-1,2))
        .andThen(makeMoveToTranslationCommand(1.5,0,2))
        .andThen(makeMoveToTranslationCommand(1,1,2))
        .andThen(makeMoveToTranslationCommand(0,2,0));
    }
    private CommandBase makeMoveToTranslationCommand(double x, double y, double targetSpeed){
        return new OdometricSwerve_MoveToTranslationCommand(
            swerve, 
            new Translation2d(x,y), 
            targetSpeed, 
            createDefaultController()).alongWith(new InstantCommand(() -> SmartDashboard.putString("MOVING TO", String.format("X: %s, Y: %s", x,y))));
    }
    private PIDController createDefaultController(){
        var controller = new PIDController(1, 0, 0);
        controller.setTolerance(0.05);
        return controller;
    }
}
