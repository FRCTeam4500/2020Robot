// package frc.robot.components.hardware;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import edu.wpi.first.wpilibj.Spark;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.SparkMax;
// import com.revrobotics.jni.*;
// import com.revrobotics.ControlType;

// import frc.robot.components.IAngleGetterComponent;
// import frc.robot.components.IAngleSetterComponent;
// import frc.robot.components.ISpeedSetterComponent;

// public class SparkComponent extends SparkMax implements ISpeedSetterComponent{
//     public static final double TICKS_PER_ROTATION = 2048;
//     public SparkComponent(int deviceNumber) {
//         super(deviceNumbe
//     }
//     @Override
//     public void setSpeed(double speed){
//         set(ControlType.kCurrent, -speed);
//     }

// }