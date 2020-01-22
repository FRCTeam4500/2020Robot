package frc.robot.subsystems.shooter;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISpeedSetterComponent;

public class Shooter extends SubsystemBase {
    private ISpeedSetterComponent topMotor;
    private ISpeedSetterComponent bottomMotor;
    public Shooter(ISpeedSetterComponent topMotor, ISpeedSetterComponent bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void run(double topSpeed, double bottomSpeed){
        this.topMotor.setSpeed(topSpeed);
        this.bottomMotor.setSpeed(bottomSpeed);
    }
}

