package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IOutputSetterComponent;

public class Shooter extends SubsystemBase {
    private IOutputSetterComponent topMotor;
    private IOutputSetterComponent bottomMotor;
    public Shooter(IOutputSetterComponent topMotor, IOutputSetterComponent bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void run(double topSpeed, double bottomSpeed){
        this.topMotor.setOutput(topSpeed);
        this.bottomMotor.setOutput(bottomSpeed);
    }
}

