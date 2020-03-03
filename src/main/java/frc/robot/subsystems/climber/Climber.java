package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.ISmartMotorComponent;
public class Climber extends SubsystemBase{
    private ISmartMotorComponent motor;
    private Servo servo;

    public Climber(ISmartMotorComponent motor, Servo servo) {
        this.motor = motor;
        this.servo = servo;
    }

    public void setSpeed(double speed) {
        motor.setOutput(speed);
    }

    public void enableServo(){
        servo.setAngle(ClimberMap.SERVO_MAX_DEGREES);
    }

    public void disableServo(){
        servo.setAngle(ClimberMap.SERVO_MIN_DEGREES);
    }

}