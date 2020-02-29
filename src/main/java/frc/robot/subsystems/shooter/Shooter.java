package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.IAngularVelocitySetterComponent;
import frc.robot.components.IOutputSetterComponent;
import frc.robot.components.ISmartMotorComponent;

public class Shooter extends SubsystemBase {
    private ISmartMotorComponent topMotor;
    private ISmartMotorComponent bottomMotor;
    private double desiredTopSpeed;
    private double desiredBottomSpeed;
    public Shooter(ISmartMotorComponent topMotor, ISmartMotorComponent bottomMotor) {
        this.topMotor = topMotor;
        this.bottomMotor = bottomMotor;
    }

    public void run(double topSpeed, double bottomSpeed){
        desiredTopSpeed = topSpeed;
        desiredBottomSpeed = bottomSpeed;
        this.topMotor.setAngularVelocity(topSpeed / 60 * 2 * Math.PI);
        this.bottomMotor.setAngularVelocity(bottomSpeed / 60 * 2 * Math.PI);
    }

    public boolean atSpeeds(double threshold){
        return Math.abs(desiredTopSpeed - radPerSecToRotPerMin(topMotor.getAngularVelocity())) < threshold && Math.abs(desiredBottomSpeed - radPerSecToRotPerMin(bottomMotor.getAngularVelocity())) < threshold;
    }
    public double radPerSecToRotPerMin(double radPerSec){
        return radPerSec / Math.PI / 2.0 * 60;
    }

}

