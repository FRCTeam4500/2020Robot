package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISpeedSetterComponent;

public class Indexer extends SubsystemBase {
    private ISpeedSetterComponent motor;
    public Indexer(ISpeedSetterComponent motor){
        this.motor = motor;
    }

    public void setSpeed(double speed){
        motor.setSpeed(speed);
    }
}
