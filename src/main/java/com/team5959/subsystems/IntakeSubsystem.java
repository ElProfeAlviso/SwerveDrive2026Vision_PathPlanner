// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;
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

  private final SparkClosedLoopController pivotPIDController;
  private final SparkClosedLoopController rollerPIDController;

  public final SparkAbsoluteEncoder absoluteEncoder;

  private final SoftLimitConfig pivotSoftLimitsConfig;

  public IntakeSubsystem() {

  pivotMotor = new SparkMax(23, MotorType.kBrushless); // Motor del pivot
  pivotMotorConfig = new SparkMaxConfig(); // Configuración del motor del pivot
  pivotMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);

  pivotMotorConfig.absoluteEncoder
  .positionConversionFactor(360.0)
  .velocityConversionFactor(360.0/60.0)
  .setSparkMaxDataPortConfig();

  pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).p(0.010).i(0.000000375).d(0.001);
  pivotMotorConfig.closedLoop.outputRange(-1, 1);

  pivotSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del pivot
  pivotSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
  pivotSoftLimitsConfig.forwardSoftLimit(215); // Posición máxima hacia adelante
  pivotSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás  
  pivotSoftLimitsConfig.reverseSoftLimit(120); // Posición máxima hacia atrás
  pivotMotorConfig.apply(pivotSoftLimitsConfig);



  pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  rollerMotor = new SparkMax(24, MotorType.kBrushless); // Motor del roller
  rollerMotorConfig = new SparkMaxConfig(); // Configuración del motor del roller
  rollerMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true);
  rollerMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);


  //rollerMotorConfig.closedLoop.pidf(0.000001, 0, 0, 0.0011);

  rollerMotorConfig.closedLoop.pidf(0.000001, 0, 0, 0.00212); 
  rollerMotorConfig.closedLoop.outputRange(-1, 1);

  rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



  absoluteEncoder = pivotMotor.getAbsoluteEncoder(); 
  pivotPIDController = pivotMotor.getClosedLoopController(); // Controlador PID del pivot//
  rollerPIDController = rollerMotor.getClosedLoopController(); // Controlador PID del roller

  
  }

  public void setPivotMotorPower(double power) {
    pivotMotor.set(power);
  }

  public void setRollerMotorPower(double power) {
    rollerMotor.set(power);
  }

  public void setPivotPIDPosition(double degrees) {
    pivotPIDController.setSetpoint(degrees, ControlType.kPosition);
  }

  public void setRollerPIDSpeed(double speed){

   rollerPIDController.setSetpoint(speed, ControlType.kVelocity);

  }

  public void stopPivotMotor() {
    pivotMotor.stopMotor();
  }

  public void stopRollerMotor() {
    rollerMotor.stopMotor();
  }

   public void holdIntakePosition(double currentPosition) {
    pivotPIDController.setReference(currentPosition, ControlType.kPosition);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Position", absoluteEncoder.getPosition());   
   SmartDashboard.putNumber("Intake Roller Speed", rollerMotor.getEncoder().getVelocity());

  }
}
