package frc.robot.subsystems.indexer.factory;

import frc.robot.components.hardware.TalonFXComponent;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerMap;

public class DefaultIndexerFactory implements IIndexerFactory{
    public Indexer makeIndexer(){
        return new Indexer(new TalonFXComponent(IndexerMap.MOTOR_PORT));
    }
}
