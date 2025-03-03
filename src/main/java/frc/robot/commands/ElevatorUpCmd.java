// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorUpCmd extends Command {
  /** Creates a new ElevatorUpCmd. */

  private final ElevatorSubSys elevatorSubSysObj;
 

  public ElevatorUpCmd(ElevatorSubSys elevatorSubSys) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.elevatorSubSysObj = elevatorSubSys;
  
    addRequirements(elevatorSubSysObj);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubSysObj.runElevUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
