// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class WinchPinSubSys extends SubsystemBase {
  /** Creates a new WinchPinSubSys. */
  private final SparkMax winchPinMotor;
  private static final Timer TIMER = new Timer();
  private final double timerLim1 = 2.0;
  private final double timerLim2 = 1.7;
  private final double downWinchVelPct;
  private final double downWinchVelVolts;
  private final double stopWinchVelVolts;
  private final double upWinchVelPct;
  private final double upWinchVelVolts;


  public WinchPinSubSys() {
    winchPinMotor = new SparkMax(Constants.WINCH_PIN_MOTOR_CAN_ID, MotorType.kBrushed);
    
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig winchPinMotorConfig = new SparkMaxConfig();



    globalConfig
      .smartCurrentLimit(5);
     // .IdleMode(IdleMode.kBrake); //TODO: look up idlemode in 
      
      winchPinMotorConfig
        .apply(globalConfig);


    downWinchVelPct = 99.0 / 100.0;
    downWinchVelVolts = downWinchVelPct * 12.0;
    upWinchVelPct = 35.0 / 100.0;
    upWinchVelVolts = upWinchVelPct * 12.0;
    stopWinchVelVolts = 0.0;
    
      
      
        }
      
        //private void IdleMode(IdleMode kbrake) {
          // TODO Auto-generated method stub
         // throw new UnsupportedOperationException("Unimplemented method 'IdleMode'");
       // }
      
        @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Winch Pin ", winchPinMotor.getAppliedOutput());

  }

  //if (TIMER.get() < timerLim2) 

  public void winchPinDown() {
    TIMER.restart();
    if (TIMER.get() < timerLim1) {
        winchPinMotor.setVoltage(-downWinchVelVolts);
    } else {
        winchPinMotor.setVoltage(0.0);

    }

  }

  public void winchPinUp() {
    TIMER.restart();
    if (TIMER.get() < timerLim2) {
        winchPinMotor.setVoltage(upWinchVelVolts);
    } else {
        winchPinMotor.setVoltage(0.0);

    }

  }
}
