package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngularVelocitySetterComponent;

public class Shooter extends SubsystemBase {
    private IAngularVelocitySetterComponent topMotor;
    private IAngularVelocitySetterComponent bottomMotor;
    public Shooter(IAngularVelocitySetterComponent topMotor, IAngularVelocitySetterComponent bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void run(double topSpeed, double bottomSpeed){
        this.topMotor.setAngularVelocity(topSpeed / 60 * 2 * Math.PI);
        this.bottomMotor.setAngularVelocity(bottomSpeed / 60 * 2 * Math.PI);
    }
}

