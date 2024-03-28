//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.MoveArm;
import frc.robot.arm.intake_shooter.intake_commands.IntakeRing;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreAmp;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreSpeaker;
import frc.robot.auto.APTBehaviors;
import frc.robot.climber.commands.Climb;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.lights.commands.SetLightsColor;
import frc.robot.utils.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  //SUBSYSTEMS
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  private final Arm arm = new Arm();

  //DRIVER CONTROLLERS
  public static CommandXboxController primaryDriver = new CommandXboxController(0);
  public static CommandXboxController secondaryDriver = new CommandXboxController(1);

  

  //SENDABLECHOOSER FOR AUTO
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {



    NamedCommands.registerCommand("IntakeRing", new IntakeRing());
    NamedCommands.registerCommand("ScoreAmp", new ScoreAmp());
    NamedCommands.registerCommand("ScoreSpeaker", new ScoreSpeaker());
    NamedCommands.registerCommand("APTBehaviors", new APTBehaviors(drivetrain));
    NamedCommands.registerCommand("Climb", new Climb());
    NamedCommands.registerCommand("SetLightsColor", new SetLightsColor());

    configureButtonBindings(); //CONFIGURE BINDINGS

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
    primaryDriver.a().whileTrue(new MoveArm());
    primaryDriver.rightTrigger().whileTrue(new ScoreSpeaker());
    primaryDriver.leftTrigger().whileTrue(new IntakeRing());


}
    


  //THIS IS ALL OF THE AUTO PLEASE DON'T WRITE AUTO ANYWHERE ELSE
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}





