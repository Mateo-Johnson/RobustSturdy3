package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
  /** Creates a new IntakeRing. */

  Solenoid writeSolenoid;
  Solenoid wrongSolenoid;

  public Climb() {
    // Use addRequirements() here to declare subsystem dependencies.

    //BASICS OF SOLENOIDS: BASICALLY, THE COMPRESSOR TAKES IN  AIR AND COMPRESSES IT. THE SOLENOIDS ARE LIKE LITTLE GATES, 
    //IF THEY ARE SET TO FALSE THEN THEY DON'T LET AIR THROUGH. THE PISTONS ARE JUST POWERED BY THAT AIR.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    writeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
    wrongSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 3);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      writeSolenoid.set(true);
      wrongSolenoid.set(true);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}



