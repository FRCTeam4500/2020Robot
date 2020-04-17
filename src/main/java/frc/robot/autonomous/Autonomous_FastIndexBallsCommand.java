/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.NotifierCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.indexer.Indexer;

/**
 * Add your docs here.
 */
public class Autonomous_FastIndexBallsCommand extends NotifierCommand {
    Intake intake;
    Arm arm;
    Indexer indexer;
    double armAngle, intakeSpeed, indexerSpeed;

    @Override
    public void initialize() {
        super.initialize();
        arm.setAngle(armAngle);
        intake.setSpeed(intakeSpeed);
        indexer.setSpeed(0.0);
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        arm.setAngle(0);
        intake.setSpeed(0);
        indexer.setSpeed(0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    private void notifierLoop() {
        if (indexer.sensor0RegistersBall() && !indexer.sensor5RegistersBall()) {
            indexer.setSpeed(indexerSpeed);
            intake.setSpeed(0);
        } else if (indexer.sensor1RegistersBall() || indexer.sensor2RegistersBall() || indexer.sensor3RegistersBall()
                || indexer.sensor4RegistersBall() || indexer.sensor5RegistersBall()) {
            indexer.setSpeed(0);
            intake.setSpeed(intakeSpeed);
        }
    }

    public Autonomous_FastIndexBallsCommand(double period, Intake intake, Arm arm, Indexer indexer, double armAngle,
            double intakeSpeed, double indexerSpeed) {
        super(() -> {}, period, arm, intake, indexer);
        m_notifier.setHandler(this::notifierLoop);
        this.intake = intake;
        this.arm = arm;
        this.indexer = indexer;
        this.armAngle = armAngle;
        this.intakeSpeed = intakeSpeed;
        this.indexerSpeed = indexerSpeed;
    }

}
