/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.pshoot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SmartDashboardPreciseShootingOI implements IPreciseShootingOI{

        public void putSmartDashboardValues(){
            SmartDashboard.putNumber("topSpeed", -1000);
            SmartDashboard.putNumber("bottomSpeed", -1000);
            SmartDashboard.putNumber("coefficient", 1);
            SmartDashboard.putNumber("threshold", 45);
        }
            
        @Override
        public double getTopSpeed() {
            return SmartDashboard.getNumber("topSpeed", 0);
        }
    
        @Override
        public double getThreshold() {
            return SmartDashboard.getNumber("threshold", 0.0);
        }
    
        @Override
        public double getCoefficient() {
            return SmartDashboard.getNumber("coefficient", 1);
        }
    
        @Override
        public double getBottomSpeed() {
            return SmartDashboard.getNumber("bottomSpeed", 0.0);
        }
    }
