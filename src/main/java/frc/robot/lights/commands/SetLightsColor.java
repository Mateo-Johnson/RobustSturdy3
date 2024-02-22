// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lights.ColorIndex;
import frc.robot.lights.Lights;

public class SetLightsColor extends Command {
  /** Creates a new ScoreAmp. */

  public SetLightsColor() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Lights.solidColor(ColorIndex.red); //SET THE LIGHTS TO RED
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
