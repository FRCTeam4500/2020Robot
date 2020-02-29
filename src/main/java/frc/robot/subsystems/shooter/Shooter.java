package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        this.topMotor.setAngularVelocity(rotPerMinToRadPerSec(topSpeed));
        this.bottomMotor.setAngularVelocity(rotPerMinToRadPerSec(bottomSpeed));
    }
    public boolean atSpeeds(double threshold){
        return  topAtSpeed(threshold) && bottomAtSpeed(threshold);
    }
    public double radPerSecToRotPerMin(double radPerSec){
        return radPerSec / Math.PI / 2.0 * 60;
    }
    public double rotPerMinToRadPerSec(double rotPerMin){
        return rotPerMin / 60 * 2 * Math.PI;
    }
    public boolean topAtSpeed(double threshold){
        return Math.abs(desiredTopSpeed - radPerSecToRotPerMin(topMotor.getAngularVelocity())) < threshold;
    }
    public boolean bottomAtSpeed(double threshold){
        return Math.abs(desiredBottomSpeed - radPerSecToRotPerMin(bottomMotor.getAngularVelocity())) < threshold;
    }
    public void disableMotors(){
        topMotor.setOutput(0);
        bottomMotor.setOutput(0);
    }
}

