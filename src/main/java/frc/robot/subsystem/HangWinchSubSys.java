// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class HangWinchSubSys extends SubsystemBase {
  /** Creates a new HangWinch. */  
    
    // hang winch motor CAN Id 7

    // Neo motor
    // hanger out is in positive volts?
    // set to brake mode TODO: use YAGSL disabled brake mode code.

    private final SparkMax hangWinchMotor;
    private final RelativeEncoder hangWinchEncoder;
    //private double hangWinchVelVolts;
    //private double hangWinchVelPct;
   
    private SparkClosedLoopController hangWinchPIDController;
    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;
    private double kMaxOutput;
    private double kMinOutput;

    private final double inWinchVelPct;
    private final double inWinchVelVolts;
    private final double endHangWinchVelPct;
    private final double endHangWinchVelVolts;
    private final double outWinchVelPct;
    private final double outWinchVelVolts;




  public HangWinchSubSys() {
    super();
    hangWinchMotor = new SparkMax(Constants.HANG_WINCH_MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig hangWinchMotorConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(40);
     // .idleMode(IdleMode.kBrake); //TODO: look up idlemode in 2025 Revlib

    hangWinchMotorConfig
      .apply(globalConfig);


    //hangWinchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
    //hangWinchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);  // TODO: might be able to go higher than 100....

    hangWinchEncoder = hangWinchMotor.getEncoder(); 
    hangWinchEncoder.setPosition(0.0);
    hangWinchPIDController = hangWinchMotor.getClosedLoopController();

   // hangWinchMotor.setSmartCurrentLimit(Constants.HANG_WINCH_MOTOR_SMART_CURRENT_LIMIT);
   // hangWinchMotor.setSecondaryCurrentLimit(Constants.HANG_WINCH_MOTOR_SECONDARY_CURRENT_LIMIT);

    inWinchVelPct = 25.0 / 100.0;
    inWinchVelVolts = inWinchVelPct * 12.0;
    outWinchVelPct = -25.0 / 100.0;
    outWinchVelVolts = outWinchVelPct * 12.0;
    endHangWinchVelPct = 0.0 / 100.0;
    endHangWinchVelVolts = endHangWinchVelPct * 12.0;

    //hangWinchMotor.setIdleMode(IdleMode.kBrake); // set to coast when needing to work on climber

  }

  public void resetEncoders() {
    hangWinchEncoder.setPosition(0.0);
  }

  public double showEncoders() {
    return hangWinchEncoder.getPosition();
  }

  // TODO: determine direction of motor. 

  public void runHangWinchOutPrep() {
    // Uncomment this for development, testing or debugging work:
    
    // PID coefficients
    kP = 0.1;
    // kI = 0.0;
    // kD = 0.0;
    // kIz = 0.0;
    // kFF = 0.0;
    kMaxOutput = 0.98;
    kMinOutput = -0.98;

    // set PID coefficients
    // hangWinchPIDController.setP(kP);
    //climberPIDController.setI(kI);
    //climberPIDController.setD(kD);
    // climberPIDController.setIZone(kIz);
   // climberPIDController.setFF(kFF);
    // hangWinchPIDController.setOutputRange(kMinOutput, kMaxOutput);

    double rotations = 25.0;  

    // hangWinchPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
    hangWinchMotor.setVoltage(outWinchVelVolts);

     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    //SmartDashboard.putNumber("Hang Winch Motor Speed", hangWinchEncoder.getVelocity());
    //SmartDashboard.putNumber("hangWinch motor volts", hangWinchMotor.getAppliedOutput());

  }



  public void runHangWinchOut() {
    // Uncomment this for development, testing or debugging work:
    //SmartDashboard.putNumber("Hang winch encoder out", hangWinchEncoder.getPosition());

    // PID coefficients
    kP = 0.1;
    // kI = 0.0;
    // kD = 0.0;
    // kIz = 0.0;
    // kFF = 0.0;
    kMaxOutput = 0.98;
    kMinOutput = -0.98;

    // set PID coefficients
    // hangWinchPIDController.setP(kP);
    //climberPIDController.setI(kI);
    //climberPIDController.setD(kD);
    // climberPIDController.setIZone(kIz);
   // climberPIDController.setFF(kFF);
    // hangWinchPIDController.setOutputRange(kMinOutput, kMaxOutput);

    double rotations = -125.0;  //was 50.0

    hangWinchPIDController.setReference(rotations, ControlType.kPosition);
    //hangWinchMotor.setVoltage(outWinchVelVolts);

     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    //SmartDashboard.putNumber("Hang Winch Motor Speed", hangWinchEncoder.getVelocity());
    //SmartDashboard.putNumber("hangWinch motor volts", hangWinchMotor.getAppliedOutput());


   
  }

  // Runs the Hang Winch Motor in a negative direction

  public void runHangWinchIn() {
    // Uncomment this for development, testing or debugging work:
    
    // PID coefficients
    kP = 0.1;
    //kI = 0.0;
    //kD = 0.0;
    //kIz = 0.0;
    //kFF = 0.0;
    kMaxOutput = 0.98;
    kMinOutput = -0.98;

    // set PID coefficients
    //hangWinchPIDController.setP(kP);
    //climberPIDController.setI(kI);
    //climberPIDController.setD(kD);
    //climberPIDController.setIZone(kIz);
    //climberPIDController.setFF(kFF);
    //hangWinchPIDController.setOutputRange(kMinOutput, kMaxOutput);

    double rotations = -1.0;

    hangWinchMotor.setVoltage(inWinchVelVolts);

    //hangWinchPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);

    // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", hangWinchEncoder.getPosition());

  }

  public void endHangWinchMotor() {
    hangWinchMotor.setVoltage(endHangWinchVelVolts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Uncomment this for development, testing or debugging work:
    SmartDashboard.putNumber("hangWinch encoder", hangWinchEncoder.getPosition());
    SmartDashboard.putNumber("hangWinch motor volts", hangWinchMotor.getAppliedOutput());

  }
}
