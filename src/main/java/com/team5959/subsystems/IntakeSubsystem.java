// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;



public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final SparkMax pivotMotor;
  private final SparkMaxConfig pivotMotorConfig;
  private final SparkMax rollerMotor;
  private final SparkMaxConfig rollerMotorConfig;

  private final SparkClosedLoopController closedLoopController;
  private final SparkAbsoluteEncoder absoluteEncoder;

  private final SoftLimitConfig pivotSoftLimitsConfig;

  public IntakeSubsystem() {

  pivotMotor = new SparkMax(23, MotorType.kBrushless); // Motor del pivot
  pivotMotorConfig = new SparkMaxConfig(); // Configuración del motor del pivot
  rollerMotor = new SparkMax(24, MotorType.kBrushless); // Motor del roller
  rollerMotorConfig = new SparkMaxConfig(); // Configuración del motor del roller

  pivotSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del pivot
  pivotSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
  pivotSoftLimitsConfig.forwardSoftLimit(215); // Posición máxima hacia adelante
  pivotSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás  
  pivotSoftLimitsConfig.reverseSoftLimit(120); // Posición máxima hacia atrás
  pivotMotorConfig.apply(pivotSoftLimitsConfig);


  
  pivotMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

  pivotMotorConfig.absoluteEncoder
  .positionConversionFactor(360.0)
  .velocityConversionFactor(360.0/60.0)
  .setSparkMaxDataPortConfig();

  pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(0.01225).i(0.00000375).d(0.05);

  pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    
 
  

   absoluteEncoder = pivotMotor.getAbsoluteEncoder(); 
  closedLoopController = pivotMotor.getClosedLoopController(); // Controlador PID del pivot//

  
  }

  public void setPivotPosition(double degrees) {
    closedLoopController.setSetpoint(degrees, ControlType.kPosition);
  }

  public void setRollerSpeed(double speed){

    rollerMotor.set(speed);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Position", absoluteEncoder.getPosition());   

  }
}
