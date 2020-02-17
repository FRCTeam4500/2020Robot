package frc.robot.subsystems.climber.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberMap;

public class DefaultClimberFactory {
    public Climber makeClimber(){
        return new Climber(new TalonSRXComponent(ClimberMap.CLIMBER_MOTOR1_PORT),
                new TalonSRXComponent(ClimberMap.CLIMBER_MOTOR2_PORT));
    }
}
