package frc.robot.subsystems.indexer.factory;

import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerMap;
import frc.robot.subsystems.indexer.NetworkTableBallSensor;

public class DefaultIndexerFactory implements IIndexerFactory{
    public final double BALL_SENSOR_THRESHOLD = 32;
    public Indexer makeIndexer(){
        return new Indexer(
            new TalonSRXComponent(IndexerMap.MOTOR_PORT),
            makeSensor(0),
            makeSensor(1),
            makeSensor(2),
            makeSensor(3),
            makeSensor(4),
            makeSensor(5)
        );
    }
    private NetworkTableBallSensor makeSensor(int sensorId){
        return new NetworkTableBallSensor("Sensor"+String.valueOf(sensorId), BALL_SENSOR_THRESHOLD);
    }
}
