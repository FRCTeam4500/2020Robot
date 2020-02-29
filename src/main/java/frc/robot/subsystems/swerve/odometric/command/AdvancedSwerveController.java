/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.utility.ExtendedMath;

/**
 * Add your docs here.
 */
public class AdvancedSwerveController {
        private double initialAllowableTranslationError;
        private double finalAllowableRotationError;
        private double deltaAllowableTranslationalErrorPerState;
        private double currentAllowableTranslationalError;
        private boolean enableRotation;
        private boolean enableTranslation;
        private double kP;
        private double kW;
        private Trajectory.State[] states;
        private Trajectory.State currentState;
        private int currentStateIndex = 0 ;
        private Translation2d axis;
        private double projectedDesiredTranslationOffset = 0;
        private double desiredRotationOffset = 0;
        private Rotation2d targetRotation;
        private double maxVelocity;
        public AdvancedSwerveController(double initialAllowableTranslationError, double finalAllowableTranslationError, boolean enableRotation, double allowableRotationError, boolean enableTranslation, double kP, double kW, Rotation2d endRotation, double maxVelocity, Trajectory.State... states){
            this.initialAllowableTranslationError = initialAllowableTranslationError;
            this.enableRotation = enableRotation;
            this.enableTranslation = enableTranslation;
            this.kP = kP;
            this.kW = kW;
            this.targetRotation = endRotation;
            this.states = states;
            this.finalAllowableRotationError = allowableRotationError;
            this.maxVelocity = maxVelocity;
            deltaAllowableTranslationalErrorPerState = (finalAllowableTranslationError - initialAllowableTranslationError
                    ) / states.length;
        }
        public AdvancedSwerveController(double initialAllowableTranslationError, double finalAllowableTranslationError, boolean enableRotation, double allowableRotationError, boolean enableTranslation, double kP, double kW, Rotation2d endRotation, double maxVelocity, Trajectory trajectory){
            this(initialAllowableTranslationError,finalAllowableTranslationError, enableRotation, allowableRotationError, enableTranslation, kP, kW, endRotation, maxVelocity, trajectory.getStates().toArray(Trajectory.State[]::new));
        }
        public double calculateTranslationOutput(Translation2d position){
            double valueToReturn = 0.0;
            if(enableTranslation)
                valueToReturn = (projectedDesiredTranslationOffset - getProjectedOffsetFromTarget(position)) * kP + currentState.velocityMetersPerSecond;
            else
                valueToReturn = 0.0;
            
            if(atCurrentStateTranslation(position) && currentStateIndex + 1 < states.length){
                    currentStateIndex++;
                    initializeCurrentState(position);
                 }

            return valueToReturn;
        }
        public double calculateRotationOutput(Rotation2d rotation){
          if(enableRotation)
            return (desiredRotationOffset + targetRotation.minus(rotation).getRadians()) * kW;
          else
            return 0.0;
        }
        public ChassisSpeeds calculateFieldCentricChassisSpeeds(Pose2d pose){
            var translationalOutput = ExtendedMath.clamp(0, maxVelocity, calculateTranslationOutput(pose.getTranslation()));
            var rotationOutput = calculateRotationOutput(pose.getRotation());
            var direction = getUnitDirectionVector(pose.getTranslation());
            return new ChassisSpeeds(translationalOutput * direction.getX(), translationalOutput * direction.getY(), rotationOutput);
        }
        public void reset(Translation2d currentTranslation){
            currentStateIndex = 0;
            currentAllowableTranslationalError = initialAllowableTranslationError + deltaAllowableTranslationalErrorPerState;
            initializeCurrentState(currentTranslation);
        }
        public boolean atCurrentStateTranslation(Translation2d currentTranslation){
            if(enableTranslation == false){
                return true;
            }else{
                if(Math.abs(projectedDesiredTranslationOffset - getProjectedOffsetFromTarget(currentTranslation)) < currentAllowableTranslationalError){
                    return true;
                }else{
                    return false;
                }
            }
        }
        public boolean atFinalStateTranslation(Translation2d currentTranslation){
            if(currentStateIndex + 1 >= states.length){
                return atCurrentStateTranslation(currentTranslation);
            }else{
                return false;
            }
        }
        public boolean atTargetRotation(Rotation2d rotation){
            if(enableRotation == false){
                return true;
            } else {
            if(Math.abs(targetRotation.minus(rotation).getRadians()) < finalAllowableRotationError){
                return true;
            }else{
                return false;
            }
            }
        }
        public boolean atFinalPose(Pose2d currentPose){
            return atFinalStateTranslation(currentPose.getTranslation()) && atTargetRotation(currentPose.getRotation());
        }
        private void initializeCurrentState(Translation2d currentTranslation){
            currentState = states[currentStateIndex];
            axis = getOffsetToCurrentState(currentTranslation);
            currentAllowableTranslationalError -= deltaAllowableTranslationalErrorPerState;
        }
        private Translation2d getOffsetFromTarget(Translation2d currentTranslation){
            return currentTranslation.minus(currentState.poseMeters.getTranslation());
        }
        private Translation2d getOffsetToCurrentState(Translation2d currentTranslation){
            return currentState.poseMeters.getTranslation().minus(currentTranslation);
        }
        private double getScalarProjectionOntoTargetAxis(Translation2d vector){
            return ExtendedMath.scalarProjectionOf(vector, axis);
          }
          private double getProjectedOffsetFromTarget(Translation2d currentTranslation){
            return getScalarProjectionOntoTargetAxis(getOffsetFromTarget(currentTranslation));
          }
        public Translation2d getUnitDirectionVector(Translation2d currentTranslation){
            return ExtendedMath.normalize(getOffsetToCurrentState(currentTranslation));
        }
    
}
