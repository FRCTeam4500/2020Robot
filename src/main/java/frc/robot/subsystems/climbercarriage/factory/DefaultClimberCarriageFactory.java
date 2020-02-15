/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climbercarriage.factory;
import frc.robot.components.hardware.TalonSRXComponent;
import frc.robot.subsystems.climbercarriage.ClimberCarriage;
import frc.robot.subsystems.climbercarriage.ClimberCarriageMap;
/**
 * Add your docs here.
 */
public class DefaultClimberCarriageFactory {
    public ClimberCarriage makeClimberCarriage(){
        return new ClimberCarriage(new TalonSRXComponent(ClimberCarriageMap.CLIMBERCARRIAGE_MOTOR1_PORT));
    }

}
