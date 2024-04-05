// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lights_deprecated.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lights_deprecated.ColorIndex;
import frc.robot.lights_deprecated.Lights;

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
    //EXAMPLE IMPLEMENTATIONS

    // double[][] colorCycle = {ColorIndex.red, ColorIndex.blue, ColorIndex.limeGreen};
    // Lights.colorCycle(colorCycle, 30);

    // Lights.colorFlash(ColorIndex.red, ColorIndex.blue, 10);

    // Lights.colorPulse(ColorIndex.red, 15);
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
