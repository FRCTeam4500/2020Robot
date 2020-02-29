/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve.odometric.command;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

/**
 * Add your docs here.
 */
public class AdvancedSwerveControllerBuilder {
    private double initialAllowableTranslationError = Double.POSITIVE_INFINITY;
    private double finalAllowableTranslationError = Double.POSITIVE_INFINITY;
    private boolean enableRotation = false;
    private double allowableRotationError = Double.POSITIVE_INFINITY;
    private boolean enableTranslation = false;
    private double kP = 0.0;
    private double kW = 0.0;
    private Rotation2d endRotation = new Rotation2d();
    private double maxVelocity = Double.POSITIVE_INFINITY;
    private Trajectory.State[] states = new Trajectory.State[0];
    public AdvancedSwerveController buildController(){
        return new AdvancedSwerveController(
            initialAllowableTranslationError, 
            finalAllowableTranslationError, 
            enableRotation, 
            allowableRotationError, 
            enableTranslation, 
            kP, 
            kW, 
            endRotation, 
            maxVelocity,
            states);
    }
    public AdvancedSwerveControllerBuilder withMaxVelocity(double maxVelocity){
        this.maxVelocity = maxVelocity;
        return this;
    }
    public AdvancedSwerveControllerBuilder withInitialAllowableTranslationError(double allowableError){
       this. initialAllowableTranslationError = allowableError;
        return this;
    }
    public AdvancedSwerveControllerBuilder withFinalAllowableTranslationError(double allowableError){
        this.finalAllowableTranslationError = allowableError;
        return this;
    }
    public AdvancedSwerveControllerBuilder withRotationsEnabled(boolean enableRotations){
        this.enableRotation = enableRotations;
        return this;
    }
    public AdvancedSwerveControllerBuilder withAllowableRotationError(double allowableError){
        this.allowableRotationError = allowableError;
        return this;
    }
    public AdvancedSwerveControllerBuilder withTranslationsEnabled(boolean enableTranslations){
        this.enableTranslation = enableTranslations;
        return this;
    }
    public AdvancedSwerveControllerBuilder with_kP(double kP){
        this.kP = kP;
        return this;
    }
    public AdvancedSwerveControllerBuilder with_kW(double kW){
        this.kW = kW;
        return this;
    }
    public AdvancedSwerveControllerBuilder withEndRotation(Rotation2d endRotation){
        this.endRotation = endRotation;
        return this;
    }
    public AdvancedSwerveControllerBuilder withTrajectoryStates(Trajectory.State... states){
        this.states = states;
        return this;
    }
    public AdvancedSwerveControllerBuilder withTrajectory(Trajectory trajectory){
        this.states = trajectory.getStates().toArray(Trajectory.State[]::new);
        return this;
    }
}
