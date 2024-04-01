// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm.intake_shooter.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.intake_shooter.shooter_commands.ScoreSpeaker;

public class PurgeRing extends Command {
  /** Creates a new Purge. */
  private final double speed;

  public PurgeRing(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ScoreSpeaker.intake1.set(speed);
    ScoreSpeaker.intake2.set(speed);
    ScoreSpeaker.rightOuttake.set(speed);
    ScoreSpeaker.leftOuttake.set(-speed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ScoreSpeaker.intake1.set(0);
    ScoreSpeaker.intake2.set(0);
    ScoreSpeaker.rightOuttake.set(0);
    ScoreSpeaker.leftOuttake.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}