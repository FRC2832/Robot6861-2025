// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {


    // Driver Cam values
    public static final int CAMERA_USB_PORT = 0;
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
    public static final int FRAMERATE = 20;

    //Coral Motor
    public static final int CORAL_MOTOR_CAN_ID = 6;
    public static final int CORAL_MOTOR_SMART_CURRENT_LIMIT = 40;

    // Winch Pin
    public static final int WINCH_PIN_MOTOR_CAN_ID = 8;

    //Hang Winch
    public static final int HANG_WINCH_MOTOR_CAN_ID = 7;
    public static final int HANG_WINCH_MOTOR_SMART_CURRENT_LIMIT = 40;
    public static final int HANG_WINCH_MOTOR_SECONDARY_CURRENT_LIMIT = 40;

    // Elevator
    public static final int ELEV_FRONT_LEADER_MOTOR_CAN_ID = 4;
    public static final int ELEV_BACK_FOLLOWER_MOTOR_CAN_ID = 5;
    
   


    public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
