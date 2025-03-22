// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.HangWinchSubSys;
//import frc.robot.subsystem.WinchPinSubSys;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WinchOutCmd extends Command {
  /** Creates a new WinchOutCmd. */
  private final HangWinchSubSys hangWinchSubSysObj;
  //private final WinchPinSubSys winchPinSubSysObj;

  private final Timer timer = new Timer();

  
  public WinchOutCmd(HangWinchSubSys hangWinchSubSysObj) {  //WinchPinSubSys winchPinSubSysObj
      // Use addRequirements() here to declare subsystem dependencies.
      this.hangWinchSubSysObj = hangWinchSubSysObj;
     // this.winchPinSubSysObj = winchPinSubSysObj;
      addRequirements(hangWinchSubSysObj); 

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  timer.reset();
  timer.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangWinchSubSysObj.runHangWinchOut();
    //winchPinSubSysObj.winchPinDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    hangWinchSubSysObj.endHangWinchMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   //if (hangWinchSubSysObj.runHangWinchOut().isHangWinchOut = true || timer.get() >= 4.5) {
     // return true;
   // } else {
     // return false;
  //  }
    return timer.get() >= 3.6;
  }
}
