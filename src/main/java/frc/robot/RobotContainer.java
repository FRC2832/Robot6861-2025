// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CoralOutCmd;
import frc.robot.commands.CoralReverseCmd;
import frc.robot.commands.CoralStopCmd;
import frc.robot.commands.ElevatorDownCmd;
import frc.robot.commands.ElevatorL4Cmd;
import frc.robot.commands.ElevatorStopCmd;
import frc.robot.commands.ElevatorUpCmd;
import frc.robot.commands.RampDownCmd;
import frc.robot.commands.RampUpCmd;
import frc.robot.commands.TurtleModeCmd;
import frc.robot.commands.SnailModeCmd;
import frc.robot.commands.WinchInCmd;
import frc.robot.commands.WinchOutCmd;
import frc.robot.commands.WinchStopCmd;
import frc.robot.leds.FrontLeds;
import frc.robot.leds.RearLeds;
import frc.robot.leds.ShowTargetInfo;
import frc.robot.ramp.RampSubsystem;
import frc.robot.subsystem.CoralSubSys;
import frc.robot.subsystem.ElevatorSubSys;
import frc.robot.subsystem.HangWinchSubSys;
import frc.robot.subsystem.WinchPinSubSys;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.vision.AprilTagCamera;
import frc.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveSubsystem swerveDrive;
    private FrontLeds frontLeds;
    private RearLeds rearLeds;
    //private Vision vision;

    private XboxController driverController;
    private XboxController operatorController;
    private WinchPinSubSys winchPinSubSysObj;
    private HangWinchSubSys hangWinchSubSysObj;
    private ElevatorSubSys elevatorSubSysObj;
    private CoralSubSys coralSubSysObj;
    private DigitalInput coralSensor;
    private SendableChooser<Command> autoChooser;

    private AprilTagCamera frontCamera;
    private UsbCamera driverCam;

    public RobotContainer() {
        driverController = new XboxController(0);
        winchPinSubSysObj = new WinchPinSubSys();
        hangWinchSubSysObj = new HangWinchSubSys();
        elevatorSubSysObj = new ElevatorSubSys();
        coralSubSysObj = new CoralSubSys();

        String swerveDirectory = "swerve/neo";
        //subsystems used in all robots
        swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveDirectory));
        frontLeds = new FrontLeds(6, 54);
        rearLeds = new RearLeds(frontLeds);
        coralSensor = new DigitalInput(9); 


        // Boilerplate code to start the camera server
        
        driverCam = CameraServer.startAutomaticCapture();
        driverCam.setResolution(640, 480);
        driverCam.setFPS(20);




        if(Robot.isSimulation()) {
            //drive fast in simulation
            swerveDrive.setMaximumSpeed(5, Math.PI);
            //start in red zone since simulation defaults to red 1 station to make field oriented easier
            swerveDrive.resetOdometry(new Pose2d(16.28, 4.03,Rotation2d.fromDegrees(180)));
        }
        else {
            swerveDrive.setMaximumSpeed(2.5, Math.PI/2); //maxspeed of robot
        }

        //vision = new Vision(swerveDrive);
        frontCamera = new AprilTagCamera("front",
            new Rotation3d(0, Units.degreesToRadians(0), Math.toRadians(0)),
            new Translation3d(0.363,
                                0,
                                0.31),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

       // vision.addCamera(frontCamera);
        /*
        vision.addCamera(new AprilTagCamera("rear",
            new Rotation3d(0, Units.degreesToRadians(-20), Math.toRadians(0)),
            new Translation3d(-0.363,
                                0,
                                0.5),
            VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)));
        */
        //add some buttons to press for development
        /*
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        */
        //SmartDashboard.putData("Test Leds", new TestLeds(leds));

        SmartDashboard.putNumber("Hang motor encoder", hangWinchSubSysObj.showEncoders());
        

        // Register Named Commands for PathPlanner
        //NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        //NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));
        NamedCommands.registerCommand("ScorePieceL1", new WaitCommand(1));
        NamedCommands.registerCommand("GetFromHP", new WaitCommand(2));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = swerveDrive.driveCommand(
            () -> MathUtil.applyDeadband(driverController.getLeftY() * -1, 0.1),
            () -> MathUtil.applyDeadband(driverController.getLeftX() * -1, 0.1),
            () -> MathUtil.applyDeadband(driverController.getRightX() * 1, 0.1));
            //() -> driverController.getRightX() * 1);  //was -1

        if (Robot.isSimulation()) {
            new Trigger(driverController::getAButton).whileTrue(swerveDrive.driveToPose(new Pose2d(11.23, 4.15, Rotation2d.fromDegrees(0))));
            new Trigger(driverController::getYButton).whileTrue(swerveDrive.driveToPose(new Pose2d(14.73, 4.49, Rotation2d.fromDegrees(120))));
        } else {
            //new Trigger(driverController::getAButton).whileTrue(swerveDrive.driveToPose(new Pose2d(2.75, 4.15, Rotation2d.fromDegrees(0))));
        }


        new Trigger(driverController::getLeftStickButton).whileTrue(swerveDrive.swerveLock());
        
        //setup default commands that are used for driving
        swerveDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        //leds.setDefaultCommand(new RainbowLeds(leds).ignoringDisable(true));
        frontLeds.setDefaultCommand(new ShowTargetInfo(frontLeds, frontCamera, Color.fromHSV(75, 255, 255)));
        rearLeds.setDefaultCommand(new ShowTargetInfo(rearLeds, frontCamera, Color.fromHSV(75, 255, 255)));
        //rearLeds.setDefaultCommand(new TestLeds(rearLeds));

        //rampSubsystem.setDefaultCommand(rampSubsystem.runMotor(() -> (driverController.getRightTriggerAxis() * 0.35) - (driverController.getLeftTriggerAxis() * 0.35)));

        // Trigger driverRightTrigger = driverController.rightTrigger();
  

        //new Trigger(operatorController::getRightTriggerAxis).whileTrue(new WinchOutCmd(winchPinSubSys));

        //new Trigger(driverController::getAButtonPressed).whileTrue(new WinchOutCmd(hangWinchSubSysObj));
        //new Trigger(driverController::getBButtonPressed).whileTrue(new WinchInCmd(hangWinchSubSysObj));
        //new Trigger(driverController::getXButtonPressed).whileTrue(new WinchStopCmd(hangWinchSubSysObj));


        // new Trigger(driverController::getAButtonPressed).whileTrue(new ElevatorDownCmd(elevatorSubSysObj));
        new Trigger(driverController::getYButtonPressed).whileTrue(new ElevatorUpCmd(elevatorSubSysObj));
        //new Trigger(driverController::getYButtonPressed).whileTrue(new ElevatorL4Cmd(elevatorSubSysObj));
        new Trigger(driverController::getXButtonPressed).whileTrue(new ElevatorStopCmd(elevatorSubSysObj));

        //new Trigger(operatorController::getYButtonPressed).whileTrue(new ElevatorUpCmd(elevatorSubSysObj));
        //new Trigger(operatorController::getYButtonPressed).whileTrue(new ElevatorL4Cmd(elevatorSubSysObj));
        //new Trigger(operatorController::getXButtonPressed).whileTrue(new ElevatorStopCmd(elevatorSubSysObj));


        //Turtle and snail mode TODO: fix this - method in command is not working
        //new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2 ).whileTrue(new TurtleModeCmd(swerveDrive));
       // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.6).whileTrue(new SnailModeCmd(swerveDrive));


        new Trigger(() -> driverController.getRightTriggerAxis() > 0.3).whileTrue(new CoralOutCmd(coralSubSysObj));
        new Trigger(() -> driverController.getRightTriggerAxis() < 0.2).whileTrue(new CoralStopCmd(coralSubSysObj));
        new Trigger(driverController::getBButtonReleased).whileTrue(new CoralReverseCmd(coralSubSysObj));

        //new Trigger(operatorController::getBButtonReleased).whileTrue(new CoralReverseCmd(coralSubSysObj));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
