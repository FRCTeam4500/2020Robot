/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.kinematic.KinematicSwerve;
import frc.robot.subsystems.swerve.kinematic.factory.ThirteenWheelSwerveFactory;

/**
 * Add your docs here.
 */
public class XboxTestRobotContainer implements IRobotContainer{
    private XboxController controller = new XboxController(0);
    private KinematicSwerve swerve = new ThirteenWheelSwerveFactory().makeSwerve();
    public XboxTestRobotContainer(){
        swerve.setDefaultCommand(new RunCommand(() -> swerve.moveRobotCentric(controller.getY(Hand.kLeft),controller.getX(Hand.kLeft), controller.getX(Hand.kRight)), swerve));
    }
}
