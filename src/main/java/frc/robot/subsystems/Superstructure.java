package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.Pivot;

public class Superstructure extends SubsystemBase {
  private final Pivot pivot;
  private final Elevator elevator;
  private final IntakeSubsystem intake;

  public enum Goals {
    DEFAULT,
    // L3_CORAL,
    L2_CORAL,
    L1_CORAL,
    INTAKE,
    // L2_ALGAE,
    // PROCESSOR,

    // CLIMBING,
    // FLOOR,
  }
  
  @AutoLogOutput(key = "Superstructure/Goal")
  private Goals currentState = Goals.DEFAULT;

  public Superstructure(Pivot pivot, Elevator elevator, IntakeSubsystem intake) {
    setName("Superstructure");
    this.pivot = pivot;
    this.elevator = elevator;
    this.intake = intake;
  }

  public void setGoal(Goals state) {
    this.currentState = state;
    switch (currentState) {
      case DEFAULT:
        pivot.setState(Pivot.State.DEFAULT);
        elevator.setState(Elevator.State.DEFAULT);
        intake.setState(IntakeSubsystem.State.HOLD);
      case L2_CORAL:
        pivot.setState(Pivot.State.SCORING);
        elevator.setState(Elevator.State.L2);
      case L1_CORAL:
        pivot.setState(Pivot.State.SCORING);
        elevator.setState(Elevator.State.L1);
      case INTAKE:
        pivot.setState(Pivot.State.INTAKE);
        elevator.setState(Elevator.State.DEFAULT);
        intake.setState(IntakeSubsystem.State.CORAL_INTAKE);
    }
  }
}
