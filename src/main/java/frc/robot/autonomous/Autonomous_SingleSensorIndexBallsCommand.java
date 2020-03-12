/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;

/**
 * Add your docs here.
 */
public class Autonomous_SingleSensorIndexBallsCommand extends CommandBase{
    private Arm arm;
    private Intake intake;
    private Indexer indexer;
    private double armAngle, intakeSpeed, indexerSpeed;

    public Autonomous_SingleSensorIndexBallsCommand(Arm arm, Intake intake, Indexer indexer, double armAngle,
            double intakeSpeed, double indexerSpeed) {
        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.armAngle = armAngle;
        this.intakeSpeed = intakeSpeed;
        this.indexerSpeed = indexerSpeed;
        addRequirements(arm, intake, indexer);
    }

    @Override
    public void initialize() {
        arm.setAngle(armAngle);
        indexer.setSpeed(0);
        intake.setSpeed(intakeSpeed);
    }

    @Override
    public void execute() {
        if(indexer.sensor5RegistersBall() != true){
            if(indexer.sensor0RegistersBall() == true){
                intake.setSpeed(0);
                indexer.setSpeed(indexerSpeed);
            }else{
                intake.setSpeed(intakeSpeed);
                indexer.setSpeed(0);
            }
        }else{
            indexer.setSpeed(0);
            intake.setSpeed(0);
        }
    }
    @Override
    public void end(boolean interrupted) {
        arm.setAngle(0);
        indexer.setSpeed(0);
        intake.setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
