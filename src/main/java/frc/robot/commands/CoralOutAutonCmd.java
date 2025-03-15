// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CoralSubSys;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutAutonCmd extends Command {
  /** Creates a new CoralOutCmd. */
  private final CoralSubSys coralSubSysObj;
  private final Timer timer = new Timer();
  private final Timer bufferTimer = new Timer();
  private boolean coralPassed;

  public CoralOutAutonCmd(CoralSubSys coralSubSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralSubSysObj = coralSubSys;
    addRequirements(coralSubSysObj);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    bufferTimer.reset();
    //coralPassed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      coralSubSysObj.runCoralMotorOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSubSysObj.stopCoral();
    timer.stop();
    bufferTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!coralPassed && !coralSubSysObj.isCoralSeen()) {
    //   coralPassed = true;
    //   bufferTimer.start();
    //   return false;
    // }
    // if (coralPassed) {
    //   return bufferTimer.get() >= 1;
    // }
    return timer.get() >= 3;
  }
}
