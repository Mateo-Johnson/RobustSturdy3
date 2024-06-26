//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.MoveArm;
import frc.robot.arm.commands.MoveArmBackwards;
import frc.robot.arm.commands.setpoints.Amp_Pos;
import frc.robot.arm.commands.setpoints.Intake_Pos;
import frc.robot.arm.commands.setpoints.SpecifiedAngle_Pos;
import frc.robot.arm.commands.setpoints.Store_Pos;
import frc.robot.arm.intake_shooter.intake_commands.IntakeRing;
import frc.robot.arm.intake_shooter.intake_commands.PurgeRing;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreSpeaker;
import frc.robot.auto.Speaker;
import frc.robot.auto.scoring.Amp;
import frc.robot.climber.commands.Climb;
import frc.robot.climber.commands.Unclimb;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.lights_deprecated.commands.SetLightsColor;
import frc.robot.utils.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {

  //SUBSYSTEMS
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  @SuppressWarnings("unused")
  private final Arm arm = new Arm(); //THIS IS USED DONT DELETE IT


  //DRIVER CONTROLLERS
  public static CommandXboxController primaryDriver = new CommandXboxController(0);
  public static CommandXboxController secondaryDriver = new CommandXboxController(1);

  public static DoubleSolenoid  wrongSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);

  

  //SENDABLECHOOSER FOR AUTO
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    //REGISTER THE COMMANDS BEFORE CREATING THE POSE ESTIMATOR
    NamedCommands.registerCommand("IntakeRing", new IntakeRing());
    NamedCommands.registerCommand("Amp", new Amp(drivetrain));
    NamedCommands.registerCommand("Speaker", new Speaker(drivetrain));
    NamedCommands.registerCommand("Climb", new Climb());
    NamedCommands.registerCommand("SetLightsColor", new SetLightsColor());

    configureButtonBindings(); //CONFIGURE BINDINGS

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PortForwarder.add(5800, "limelight-arm.local", 5800);
    PortForwarder.add(5800, "limelight-front.local", 5800);

    ///CONFIGURE DEFAULT COMMANDS
    drivetrain.setDefaultCommand(

        //LEFT STICK IS TRANSLATION RIGHT STICK IS TURNING
      new RunCommand(() -> drivetrain.drive(
        -MathUtil.applyDeadband(primaryDriver.getLeftY(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT X SPEED
        -MathUtil.applyDeadband(primaryDriver.getLeftX(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT Y SPEED
        -MathUtil.applyDeadband(primaryDriver.getRightX(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT ROTATION
        false, true),
    drivetrain));
  }




  private void configureButtonBindings() {
    //DEFINE ALL OF THE BUTTON BINDINGS HERE PLEASE AND THANKS

    //SHOOTING AND INTAKING
    secondaryDriver.rightTrigger().whileTrue(new ScoreSpeaker());
    secondaryDriver.leftTrigger().whileTrue(new IntakeRing());
    secondaryDriver.leftBumper().whileTrue(new PurgeRing(0.5));

    //ARM PRESETS
    secondaryDriver.povUp().toggleOnTrue(new SpecifiedAngle_Pos(75));
    secondaryDriver.povDown().toggleOnTrue(new Intake_Pos());
    secondaryDriver.povRight().toggleOnTrue(new Store_Pos());
    secondaryDriver.povLeft().toggleOnTrue(new Amp_Pos());

    //ARM MANUAL MOVEMENT
    secondaryDriver.a().whileTrue(new MoveArm());
    secondaryDriver.b().whileTrue(new MoveArmBackwards());

    //CLIMB
    primaryDriver.y().whileTrue(new Climb());
    primaryDriver.x().whileTrue(new Unclimb());

}
    


  //THIS IS ALL OF THE AUTO PLEASE DON'T WRITE AUTO ANYWHERE ELSE
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}





