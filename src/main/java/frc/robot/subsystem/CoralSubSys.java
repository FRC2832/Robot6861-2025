// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class CoralSubSys extends SubsystemBase {
  /** Creates a new CoralSubsys. */

   private final SparkMax coralMotor;
   private final RelativeEncoder coralMotorEncoder;
   private final double coralVelVolts;
   private final double coralVelPct;
   //private final double coralVelRPM;
   private final double stopCoralVelVolts;
   private final double stopCoralVelPct;
   DigitalInput coralSensor;
   boolean coralSeen;

    //private SparkPIDController coralPIDController = new SparkPIDController();
    


    private double kPFr;
    private double kIFr;
    private double kDFr;
    private double kIzFr;
    private double kFFFr;
    private double kMaxOutputFR;
    private double kMinOutputFR;
    private double maxRPM;


  public CoralSubSys(DigitalInput coralSensor) {
    super();
   
      coralMotor = new SparkMax(Constants.CORAL_MOTOR_CAN_ID, MotorType.kBrushless);
      this.coralSensor = coralSensor;

      SparkMaxConfig globalConfig = new SparkMaxConfig();
      SparkMaxConfig coralMotorConfig = new SparkMaxConfig();

      globalConfig
        .smartCurrentLimit(60);
         //.idleMode(IdleMode.kBrake);  TODO: figure out why this isn't working

      coralMotorConfig
          .apply(globalConfig);
        

        //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
        //coralMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
        
        //coralMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
        
        coralMotorEncoder = coralMotor.getEncoder();
        
        coralMotorEncoder.setPosition(0.0);
       
        //coralPIDController = coralMotor.getPIDController();
        
    

        coralVelPct = 95.0 / 100.0;
        coralVelVolts = coralVelPct * 12.0;

        stopCoralVelPct = 0.0 / 100.0;
        stopCoralVelVolts = stopCoralVelPct * 12.0;

        //TODO: figure out the 2025 version of this coralMotor.setIdleMode(IdleMode.kBrake); 
        
    }

  public void resetEncoders() {
        coralMotorEncoder.setPosition(0.0);
        
    }

  public double showEncoders() {
      return coralMotorEncoder.getPosition();
    }


  public void runCoralMotorOut() {

         // Uncomment this for development, testing or debugging work:
         SmartDashboard.putNumber("coral Motor Speed", coralMotorEncoder.getVelocity());

        
        //shooterMotorFR.set(Constants.FR_SHOOTER_MOTOR_PCT_AMP / 100.0);


        // PID FR  Motor coefficients  TODO - change these for Coral when ready
         kPFr = 0.00000;  //was .0256
         //KiFr = 0.0;
         //KdFr = 0.0; 
         //KizFr = 0.0; 
         // Kf = 0.00017;  //was 0.00013
         kMaxOutputFR = 0.98; 
         kMinOutputFR = -0.98;
         maxRPM = 5676.0;  // from REV data sheet


        // set PID coefficients FR motor
        //shooterPIDControllerFR.setP(lowKpFr);
        //shooterPIDControllerFR.setI(lowKiFr);
        //shooterPIDControllerFR.setD(lowKdFr);
        //shooterPIDControllerFR.setIZone(lowKizFr);
        //shooterPIDControllerFR.setFF(lowKffFr);
        //shooterPIDControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);


        
        double lowFlRPM =  -3500.0;    //was -3000.0
        double lowFrRPM =  3500.0;     //was 3000.0

        //shooterPIDControllerFL.setReference(lowFlRPM, CANSparkMax.ControlType.kVelocity);
        //shooterPIDControllerFR.setReference(lowFrRPM, CANSparkMax.ControlType.kVelocity);

        coralMotor.setVoltage(coralVelVolts);


        //SmartDashboard.putNumber("RPM FL Shooter", lowFlRPM);
       // SmartDashboard.putNumber("RPM FR Shooter", lowFrRPM);
        //SmartDashboard.putNumber("ProcessVariable Shooter FL", shooterMotorFLEncoder.getVelocity());
       // SmartDashboard.putNumber("ProcessVariable Shooter FR", shooterMotorFREncoder.getVelocity());

        //SmartDashboard.putNumber("FL rpm for amp", shooterMotorFLEncoder.getVelocity());
        //SmartDashboard.putNumber("FR rpm for amp", shooterMotorFREncoder.getVelocity());

        SmartDashboard.putNumber("Coral Motor Speed", coralMotorEncoder.getVelocity());
        SmartDashboard.putNumber("Coral motor volts", coralMotor.getAppliedOutput());
       
  }




  public void runShooterHighSpeed() {

        //shooterMotorFR.setVoltage(shooterVelVoltsFR);  // comment this out when running PID
        // shooterMotorFL.setVoltage(shooterVelVoltsFL);  // comment this out when running PID

         // PID FL  Motor coefficients
         //kPFl = 0.00024; //was 0.0016
         //kIFl = 0.0;
         //kDFl = 0.0; 
         //kIzFl = 0.0; 
         //kFFFl = 0.00017;   //was 0.000179
         //kMaxOutputFL = 0.99; 
         //kMinOutputFL = -0.98;
         //maxRPM = 5676.0;  // from REV data sheet

        // set PID coefficients FL motor
        //shooterPIDControllerFL.setP(kPFl);
        //shooterPIDControllerFL.setI(kIFl);
        //shooterPIDControllerFL.setD(kDFl);
        //shooterPIDControllerFL.setIZone(kIzFl);
        //shooterPIDControllerFL.setFF(kFFFl);
        //shooterPIDControllerFL.setOutputRange(kMinOutputFL, kMaxOutputFL);

        // PID FR  Motor coefficients
         //kPFr = 0.00024;  //6e-5;   // REV suggested value. May need to change for our motors
         //kIFr = 0.0;
        // kDFr = 0.0; 
         //kIzFr = 0.0; 
         //kFFFr = 0.00017; // was 0.000179 REV suggested value. May need to change for our motors
         //kMaxOutputFR = 0.99; 
         //kMinOutputFR = -0.98;
         //maxRPM = 5676.0;  // from REV data sheet


        // set PID coefficients FR motor
        //shooterPIDControllerFR.setP(kPFr);
        //shooterPIDControllerFR.setI(kIFr);
        //shooterPIDControllerFR.setD(kDFr);
        //shooterPIDControllerFR.setIZone(kIzFr);
        //shooterPIDControllerFR.setFF(kFFFr);
        //shooterPIDControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);


        
        double flRPM =  -5350.0;  // TODO: was 5320.  Might need 2 speeds for far and away. get encoder values from smartdashboard
        double frRPM =  5300.0;  // TODO: was 5205.  get encoder values from smartdashboard

        //coralPIDController.setReference(frRPM, SparkMax.ControlType.kVelocity);


        //SmartDashboard.putNumber("RPM Coral", frRPM);
        //SmartDashboard.putNumber("ProcessVariable Coral", coralMotorEncoder.getVelocity());
       // Logger.registerCanSparkMax("Shooter Motor FL", () -> getVelocity());
       // Logger.registerCanSparkMax("Shooter Motor FR", shooterMotorFREncoder.getVelocity());
  }


  public void runCoralReverse() {
        double coralVelVoltsReverse = -0.75 * 12.0;
      
        coralMotor.setVoltage(coralVelVoltsReverse);
  }
  

  public void stopCoral() {
        coralMotor.setVoltage(stopCoralVelVolts);
        
        
        //coralMotor.setIdleMode(IdleMode.kBrake); 
  }


  public boolean isCoralSeen() {
    return !coralSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //coralSeen = !coralSensor.get();
    SmartDashboard.putBoolean("CoralSensor", !coralSensor.get());
    //SmartDashboard.putBoolean("CoralSensor", coralSeen);

    SmartDashboard.putNumber("Coral Motor Speed", coralMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Coral motor volts", coralMotor.getAppliedOutput());
       
  }
}
