package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.Pivot;

public class Superstructure extends SubsystemBase {
  private final Pivot pivot;
  private final Elevator elevator;
  private final IntakeSubsystem intake;

  public enum States {
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

  private States currentState = States.DEFAULT;
  
    public Superstructure(Pivot pivot, Elevator elevator, IntakeSubsystem intake) {
      this.pivot = pivot;
      this.elevator = elevator;
      this.intake = intake;
  }

  public void setGoal(States state) {
    this.currentState = state;
    switch (currentState) {
      case DEFAULT:
        pivot.setState(Pivot.State.DEFAULT);
        elevator.setState(Elevator.State.DEFAULT);
        intake.setState(IntakeSubsystem.State.ALGAE_HOLD);
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
