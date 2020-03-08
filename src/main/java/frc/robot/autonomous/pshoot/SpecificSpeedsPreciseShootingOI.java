/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.pshoot;

/**
 * Add your docs here.
 */
public class SpecificSpeedsPreciseShootingOI implements IPreciseShootingOI{
        private double topSpeed;
        private double bottomSpeed;
        private double coefficient;
        private double threshold;
    
        @Override
        public double getTopSpeed() {
          return topSpeed;
        }
    
        @Override
        public double getBottomSpeed() {
          return bottomSpeed;
        }
    
        @Override
        public double getCoefficient() {
          return coefficient;
        }
    
        @Override
        public double getThreshold() {
          return threshold;
        }
    
        public SpecificSpeedsPreciseShootingOI(double topSpeed, double bottomSpeed, double coefficient, double threshold) {
          this.topSpeed = topSpeed;
          this.bottomSpeed = bottomSpeed;
          this.coefficient = coefficient;
          this.threshold = threshold;
        }
}
