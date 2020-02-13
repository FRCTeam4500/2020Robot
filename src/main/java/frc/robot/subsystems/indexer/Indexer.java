package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IOutputSetterComponent;

public class Indexer extends SubsystemBase {
    private IOutputSetterComponent motor;
    IBallSensor sensor0, sensor1, sensor2, sensor3, sensor4, sensor5;
    public Indexer(IOutputSetterComponent motor, IBallSensor sensor0, IBallSensor sensor1, IBallSensor sensor2, IBallSensor sensor3, IBallSensor sensor4, IBallSensor sensor5){
        this.motor = motor;
        this.sensor0 = sensor0;
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
        this.sensor3 = sensor3;
        this.sensor4 = sensor4;
        this.sensor5 = sensor5;
    }

    public void setSpeed(double speed){
        motor.setOutput(speed);
    }
    public boolean sensor0RegistersBall(){
        return sensor0.registersBall();
    }
    public boolean sensor1RegistersBall(){
        return sensor1.registersBall();
    }
    public boolean sensor2RegistersBall(){
        return sensor2.registersBall();
    }
    public boolean sensor3RegistersBall(){
        return sensor3.registersBall();
    }
    public boolean sensor4RegistersBall(){
        return sensor4.registersBall();
    }
    public boolean sensor5RegistersBall(){
        return sensor5.registersBall();
    }
}
