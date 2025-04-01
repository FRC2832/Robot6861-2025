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

import edu.wpi.first.wpilibj.XboxController;
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

  private final double upElevFrontLeaderVelPct;
  private final double upElevVelVolts;
  private final double downElevFrontLeaderVelPct;
  private final double downElevVelVolts;
  private final double holdElevFrontLeaderVelPct;
  private final double holdElevVelVolts;
  private final double stopElevVelVolts;
  private final double stopElevVelPct;




  public ElevatorSubSys() {
    super();
    elevFrontLeaderMotor = new SparkMax(Constants.ELEV_FRONT_LEADER_MOTOR_CAN_ID, MotorType.kBrushless);
    elevBackFollowerMotor = new SparkMax(Constants.ELEV_BACK_FOLLOWER_MOTOR_CAN_ID, MotorType.kBrushless);
    
    //elevFrontLeaderhMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
    //elevFrontLeaderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);  // TODO: might be able to go higher than 100....

    elevFrontLeaderPIDController = elevFrontLeaderMotor.getClosedLoopController();
    elevFrontLeaderEncoder = elevFrontLeaderMotor.getEncoder();

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig elevFrontLeaderMotorConfig = new SparkMaxConfig();
    SparkMaxConfig elevBackFollowerMotorConfig = new SparkMaxConfig();

    globalConfig
      .smartCurrentLimit(70) 
      .idleMode(IdleMode.kBrake); 


    elevFrontLeaderMotorConfig
      .apply(globalConfig)
      .inverted(false);

    
    elevBackFollowerMotorConfig
      .apply(globalConfig)
      .follow(elevFrontLeaderMotor)
      .inverted(false);

    
    elevFrontLeaderMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.09, ClosedLoopSlot.kSlot0)
      .i(0.0, ClosedLoopSlot.kSlot0)
      .d(0.0, ClosedLoopSlot.kSlot0)
      .outputRange(-0.4, 0.3, ClosedLoopSlot.kSlot0); //was 0.3


    
    
    elevFrontLeaderMotor.configure(elevFrontLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   // Look into how to what reset safe parameters means (answer from docs: useful in case the SPARK MAX is replaced)
    elevBackFollowerMotor.configure(elevBackFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    elevFrontLeaderEncoder.setPosition(0.0);
    
    elevBackFollowerEncoder = elevBackFollowerMotor.getEncoder();
    elevBackFollowerEncoder.setPosition(0.0);


    upElevFrontLeaderVelPct = -35.0 / 100.0; // was -21.0
    upElevVelVolts = upElevFrontLeaderVelPct * 12.0;
    downElevFrontLeaderVelPct = -0.25 / 100.0;  //was -0.5 needs small negative value to counteract effect of gravity. kG = 1.2 volts
    downElevVelVolts = downElevFrontLeaderVelPct * 12.0;
    holdElevFrontLeaderVelPct = -11.0 / 100.0; // was -10
    holdElevVelVolts = holdElevFrontLeaderVelPct * 12.0;
    stopElevVelPct = 0.0 / 100.0;
    stopElevVelVolts = stopElevVelPct * 12.0;


  }

  public void resetEncoders() {
    elevFrontLeaderEncoder.setPosition(0.0);
  }

  public double showEncoders() {
    return elevFrontLeaderEncoder.getPosition();
  }


  public void runElevUpJoystick(double leftJoystickValue) {

    //double rotations = 5.0;  

    //elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    elevFrontLeaderMotor.setVoltage(upElevVelVolts * -leftJoystickValue);  //needs to be net negative because -volts is up for elevator
   
  }

  public void runElevHold() {

    //double rotations = 15.0;  

    // elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //if (elevFrontLeaderEncoder.getPosition() > 2) { // TODO: test value
    
    elevFrontLeaderMotor.setVoltage(holdElevVelVolts);

    //} else {
      //elevFrontLeaderMotor.setVoltage(0);
    //}
    

     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    //SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
    //SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Back Follower", elevBackFollowerMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Front Leader", elevFrontLeaderMotor.getAppliedOutput());
   
  }




  public void runElevL4() {

    double rotations = -23.0;

    elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);



     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //martDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());
    //SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
   // SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Back Follower volts", elevBackFollowerMotor.getAppliedOutput());
   

   
  }


  public void runElevL2() {

    double rotations = -6.0;

    elevFrontLeaderPIDController.setReference(
                                rotations, 
                                ControlType.kMAXMotionPositionControl, 
                                ClosedLoopSlot.kSlot0,
                                -0.8);  //actually need like 1.2 but testing low for now



     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //martDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());
    //SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
   // SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
    //SmartDashboard.putNumber("Back Follower volts", elevBackFollowerMotor.getAppliedOutput());
   

   
  }


  public void runElevUp() {
    // Uncomment this for development, testing or debugging work:
   
    //double rotations = 5.0;  //should be around 15

   // elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    elevFrontLeaderMotor.setVoltage(upElevVelVolts);

     // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
    //SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
   // SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
   // SmartDashboard.putNumber("Back Follower", elevBackFollowerMotor.getAppliedOutput());
   // SmartDashboard.putNumber("Front Leader", elevFrontLeaderMotor.getAppliedOutput());
   
  }

 
  public void runElevBottom() {

    double rotations = -10.0;

    elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);


    // Voltage open loop control
    // if (elevFrontLeaderEncoder.getPosition() > 4.5 || elevFrontLeaderEncoder.getPosition() < 6.0) { //TODO - change to 14.5 and 16.0 when working 
       // elevFrontLeaderMotor.setVoltage(holdElevVelVolts);
   // } else {

       // elevFrontLeaderMotor.setVoltage(upElevVelVolts);
    //}
       

     // Uncomment these for development, testing or debugging work:
   // SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());
    //SmartDashboard.putNumber("Elevator up Motor Speed", elevFrontLeaderEncoder.getVelocity());
   // SmartDashboard.putNumber("Elevator up motor volts", elevFrontLeaderMotor.getAppliedOutput());
  //  SmartDashboard.putNumber("Back Follower volts", elevBackFollowerMotor.getAppliedOutput());
   

   
  }



  public void runElevDown(double leftJoystickValue) {
    // Uncomment this for development, testing or debugging work:

    //double rotations = -1.0;

    elevFrontLeaderMotor.setVoltage(downElevVelVolts * -leftJoystickValue);  //needs to be net positive because for elevator, -volts is up, +volts is down

    // elevFrontLeaderPIDController.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    // Uncomment these for development, testing or debugging work:
    //SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", elevFrontLeaderEncoder.getPosition());

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
    //SmartDashboard.putNumber("Elevator Front Leader percent", elevFrontLeaderMotor.getAppliedOutput());

  }
}