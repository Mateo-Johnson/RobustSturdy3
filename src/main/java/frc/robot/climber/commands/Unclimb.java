package frc.robot.climber.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.arm.Arm;

public class Unclimb extends Command {
  /** Creates a new IntakeRing. */
  DoubleSolenoid solenoid;

  public static boolean extended;

  public Unclimb() {
    // Use addRequirements() here to declare subsystem dependencies.

    //BASICS OF SOLENOIDS: BASICALLY, THE COMPRESSOR TAKES IN  AIR AND COMPRESSES IT. THE SOLENOIDS ARE LIKE LITTLE GATES, 
    //IF THEY ARE SET TO FALSE THEN THEY DON'T LET AIR THROUGH. THE PISTONS ARE JUST POWERED BY THAT AIR.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    solenoid = RobotContainer.wrongSolenoid;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
    Arm.rotateVector(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    solenoid.set(DoubleSolenoid.Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}



