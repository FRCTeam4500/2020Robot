package frc.robot.subsystems.Intake.factory;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.components.hardware.SparkMaxComponent;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeMap;

public class HardwareIntakeFactory implements IIntakeFactory {
    /**
     *
     */
    ;

    public Intake makeIntake(){
        return  new Intake(new SparkMaxComponent(IntakeMap.INTAKE_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless));
    }
}
