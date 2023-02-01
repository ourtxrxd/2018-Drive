package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive driveSubsystem;
  
    private DoubleSupplier leftInput, rightInput;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(Drive driveSubsystem, DoubleSupplier leftInput, DoubleSupplier rightInput) {
      this.driveSubsystem = driveSubsystem;
      this.leftInput = leftInput;
      this.rightInput = rightInput;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveSubsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      this.driveSubsystem.arcadeDrive(
        this.leftInput.getAsDouble(),
        this.rightInput.getAsDouble()
      );
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      this.driveSubsystem.tankDrive(
        0,
        0
      );
  
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }