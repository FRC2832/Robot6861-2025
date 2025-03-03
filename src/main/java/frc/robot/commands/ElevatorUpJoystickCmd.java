// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorUpJoystickCmd extends Command {
  /** Creates a new EleveatorUpJoystickCmd. */
   private final XboxController operatorController;
   private final ElevatorSubSys elevatorSubSysObj;
 

  public ElevatorUpJoystickCmd(ElevatorSubSys elevatorSubSys, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubSysObj = elevatorSubSys;
    this.operatorController = operatorController;
    addRequirements(elevatorSubSysObj);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {

    elevatorSubSysObj.runElevUpJoystick(operatorController.getLeftY());
  }
  
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubSysObj.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
