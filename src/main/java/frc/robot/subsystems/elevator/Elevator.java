package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public enum State {
    DEFAULT(ElevatorConstants.DEFAULT_POSITION),
    L1(ElevatorConstants.L1_SCORE_POSITION),
    L2(ElevatorConstants.L2_SCORE_POSITION);

    public final double position;

    State(double position) {
      this.position = position;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private State currentState = State.DEFAULT;

  private boolean atSetpoint = true;

  public Elevator(ElevatorIO io) {
    this.io = io;
    setName("Elevator");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(State state) {
    currentState = state;
    io.setPosition(currentState.position);
  }

  @AutoLogOutput(key = "Elevator/State")
  public State getState() {
    return currentState;
  }

  @AutoLogOutput(key = "Elevator/At Setpoint")
  public boolean isAtSetpoint() {
    atSetpoint =
        Math.abs(Units.radiansToRotations(inputs.positionRads) - currentState.position)
            <= ElevatorConstants.SETPOINT_TOLERANCE;

    return atSetpoint;
  }
}
