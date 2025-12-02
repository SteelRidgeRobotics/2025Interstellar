package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public enum State {
    HOLD(0, false),
    ALGAE_HOLD(IntakeConstants.ALGAE_HOLD, true),
    CORAL_INTAKE(IntakeConstants.CORAL_INTAKE_SPEED, false),
    CORAL_OUTPUT(IntakeConstants.CORAL_OUTPUT_SPEED, true),
    ALGAE_INTAKE(IntakeConstants.ALGAE_INTAKE_SPEED, false),
    ALGAE_OUTPUT(IntakeConstants.ALGAE_OUTPUT_SPEED, true),
    L1_OUTPUT(IntakeConstants.L1_OUTPUT_SPEED, true);

    public final double output;
    public final boolean ignoreLimits;

    State(double output, Boolean ignoreLimits) {
      this.output = output;
      this.ignoreLimits = ignoreLimits;
    }
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert topintakeDisconnectAlert;
  private final Alert bottomIntakeDisconnecAlert;

  private State currentState = State.HOLD;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    setName("Intake");

    topintakeDisconnectAlert = new Alert("Top intake motor disconnected", AlertType.kError);
    bottomIntakeDisconnecAlert = new Alert("Bottom intake motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    topintakeDisconnectAlert.set(!inputs.topIntakeConnected);
    bottomIntakeDisconnecAlert.set(!inputs.bottomIntakeConnected);
  }

  @AutoLogOutput(key = "Intake/State")
  public State getState() {
    return currentState;
  }

  public void setState(State state) {
    currentState = state;
    boolean ignoringLimits = state.ignoreLimits;
    Logger.recordOutput("Intake/Ignoring Limit Switch", ignoringLimits);
    io.setIntakeOpenLoop(currentState.output, ignoringLimits);
  }

  public Command setDesiredStateCommand(State state) {
    return new InstantCommand(() -> setState(state), this);
  }

  public boolean hasCoral() {
    return inputs.sensorSensed;
  }
}
