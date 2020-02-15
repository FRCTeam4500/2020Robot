/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;
import frc.robot.subsystems.climber.IClimberOI;
import frc.robot.components.ISpeedSetterComponent;
import frc.robot.subsystems.climber.Climber;
public class ClimberMotor implements ISpeedSetterComponent {
    private ISpeedSetterComponent motor;
    public void Intake(ISpeedSetterComponent motor){
        this.motor = motor;
    }
    @Override
    public void setSpeed(double speed) {
        motor.setSpeed(speed);
    }
}
