// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

public class ElevatorSubSys extends SubsystemBase {
  /** Creates a new ElevatorSubSys. */
  private final SparkMax elevFrontLeaderMotor;
  private final SparkMax elevBackFollowerMotor;
  private final RelativeEncoder elevFrontLeaderEncoder;
  private final RelativeEncoder elevBackFollowerEncoder;

  private SparkClosedLoopController elevFrontLeaderPIDController;
    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;
    private double kMaxOutput;
    private double kMinOutput;

    private final double upElevFrontLeaderVelPct;
    private final double upElevVelVolts;
    private final double downElevFrontLeaderVelPct;
    private final double downElevVelVolts;
    private final double stopElevVelVolts;
    private final double stopElevVelPct;




  public ElevatorSubSys() {
    elevFrontLeaderMotor = new SparkMax(Constants.ELEV_FRONT_LEADER_MOTOR_CAN_ID, MotorType.kBrushless);
    elevBackFollowerMotor = new SparkMax(Constants.ELEV_BACK_FOLLOWER_MOTOR_CAN_ID, MotorType.kBrushless);
    
    //hangWinchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
    //hangWinchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);  // TODO: might be able to go higher than 100....

    elevFrontLeaderPIDController = elevFrontLeaderMotor.getClosedLoopController(); // or does it need to be closedLoopController on the left?
    elevFrontLeaderEncoder = elevFrontLeaderMotor.getEncoder();

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig elevFrontLeaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig elevBackFollowerMotorConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(60) // TODO: increase/change number later
      .idleMode(IdleMode.kCoast);  //TODO: might need to set to kBrake if elevator moves up and down during competition
    
    elevFrontLeaderMotorConfig
      .apply(globalConfig)
      .inverted(true);

    
    elevBackFollowerMotorConfig
      .apply(globalConfig)
      .follow(elevFrontLeaderMotor)
      .inverted(false);

    
    elevFrontLeaderMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(kP)
      .i(kI)
      .d(kD)
      .outputRange(kMinOutput, kMaxOutput);


    
    
    elevFrontLeaderMotor.configure(globalConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);   //TODO: Look into how to what reset safe parameters means
    elevBackFollowerMotor.configure(elevBackFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    elevFrontLeaderEncoder.setPosition(0.0);
    
    elevBackFollowerEncoder = elevBackFollowerMotor.getEncoder();
    elevBackFollowerEncoder.setPosition(0.0);

   // hangWinchMotor.setSmartCurrentLimit(Constants.HANG_WINCH_MOTOR_SMART_CURRENT_LIMIT);
   // hangWinchMotor.setSecondaryCurrentLimit(Constants.HANG_WINCH_MOTOR_SECONDARY_CURRENT_LIMIT);

    upElevFrontLeaderVelPct = -60.0 / 100.0;
    upElevVelVolts = upElevFrontLeaderVelPct * 12.0;
    downElevFrontLeaderVelPct = 7.0 / 100.0;
    downElevVelVolts = downElevFrontLeaderVelPct * 12.0;
    stopElevVelPct = 0.0 / 100.0;
    stopElevVelVolts = stopElevVelPct * 12.0;


  }

  public void resetEncoders() {
    elevFrontLeaderEncoder.setPosition(0.0);
  }

  public double showEncoders() {
    return elevFrontLeaderEncoder.getPosition();
  }



  public void runElevL4() {
    // Uncomment this for development, testing or debugging work:
    SmartDashboard.putNumber("Elevator front leader encoder L4", elevFrontLeaderEncoder.getPosition());

   

    // PID coefficients
    kP = 0.4;
    kI = 0.0;
    kD = 0.0;
    // kIz = 0.0;
    // kFF = 0.0;
    kMaxOutput = 0.98;
    kMinOutput = -0.98;

    
    double targetPosition = 15.0;  

    elevFrontLeaderPIDController.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);


    // Voltage open loop control
    //elevFrontLeaderMotor.setVoltage(upElevVelVolts);

     // Uncomment these for development, testing or debugging work:
    SmartDashboard.putNumber("SetPoint", targetPosition);
    SmartDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());
    SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Back Follower volts", elevBackFollowerMotor.getAppliedOutput());
   

   
  }


  public void runElevUp() {
    // Uncomment this for development, testing or debugging work:
    SmartDashboard.putNumber("Elevator front leader encoder up", elevFrontLeaderEncoder.getPosition());

    // PID coefficients
    kP = 0.74;
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

    double rotations = 50.0;  

    // hangWinchPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
    elevFrontLeaderMotor.setVoltage(upElevVelVolts);

     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Back Follower", elevBackFollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Front Leader", elevFrontLeaderMotor.getAppliedOutput());
   
  }

  // Runs the Hang Winch Motor in a negative direction

  public void runElevDown() {
    // Uncomment this for development, testing or debugging work:
    SmartDashboard.putNumber("Elev front motor encoder - in", elevFrontLeaderEncoder.getPosition());

    // PID coefficients
    kP = 0.5;
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

    elevFrontLeaderMotor.setVoltage(downElevVelVolts);

    //hangWinchPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);

    // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());

  }

  //public void endelevMotor() {
    //elevFrontLeaderMotor.setVoltage(upElevVelVolts);
  //}
  public void stopElevator() {
    elevFrontLeaderMotor.setVoltage(stopElevVelVolts);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Uncomment this for development, testing or debugging work:
    SmartDashboard.putNumber("Elevator Front Leader encoder", elevFrontLeaderEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Front Leader volts", elevFrontLeaderMotor.getAppliedOutput());

  }
}