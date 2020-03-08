/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.pshoot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.autonomous.VisionDistanceCalculator;

/**
 * Add your docs here.
 */
public class VisionPreciseShootingOI implements IPreciseShootingOI{

    VisionDistanceCalculator visionDistanceCalculator;

    public VisionPreciseShootingOI(VisionDistanceCalculator visionDistanceCalculator){
        this.visionDistanceCalculator = visionDistanceCalculator;
    }
            
    @Override
    public double getTopSpeed() {
        double distance = Units.metersToFeet(visionDistanceCalculator.getDistanceFromTargetMeters());
        return -12043 + 1244*distance - 60.7 *distance * distance + .905 * distance * distance * distance;
    }

    @Override
    public double getThreshold() {
        return 500;
    }

    @Override
    public double getCoefficient() {
        return 1;
    }

    @Override
    public double getBottomSpeed() {
        double distance = Units.metersToFeet(visionDistanceCalculator.getDistanceFromTargetMeters());
        return -7179 + 466*distance - 18.1*distance*distance + .211 * distance * distance * distance;
    }
}
