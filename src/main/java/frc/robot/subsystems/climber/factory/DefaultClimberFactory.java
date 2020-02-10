package frc.robot.subsystems.climber.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMap;

public class DefaultClimberFactory {
    public Climber makeClimber(){
        return new Climber(new TalonFXComponent(ClimberMap.CLIMBER_MOTOR1_PORT),
                new TalonFXComponent(ClimberMap.CLIMBER_MOTOR2_PORT));
    }
}
