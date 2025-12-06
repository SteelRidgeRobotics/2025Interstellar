package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public enum State{
        HOLD(0, false),
        ALGAE_HOLD(IntakeConstants.ALGAE_HOLD, true),
        CORAL_INTAKE(IntakeConstants.CORAL_INTAKE_SPEED, false),
        CORAL_OUTPUT(IntakeConstants.CORAL_OUTPUT_SPEED, true),
        ALGAE_INTAKE(IntakeConstants.ALGAE_INTAKE_SPEED, false),
        ALGAE_OUTPUT(IntakeConstants.ALGAE_OUTPUT_SPEED, true),
        L1_OUTPUT(IntakeConstants.L1_OUTPUT_SPEED, true);

        public final double output;
        public final boolean ignoreLimits;

        State(double output, boolean ignoreLimits){
            this.output = output;
            this.ignoreLimits = ignoreLimits;


        }

    }
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert coralIntakeDisconnectAlert;
  private final Alert coral2IntakeDisconnectAlert;
  private final Alert algaeIntakeDisconnectAlert;

    private State currentState = State.HOLD;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        setName("Intake");

    coralIntakeDisconnectAlert = new Alert("Coral intake motor disconnected", AlertType.kError);
    coral2IntakeDisconnectAlert = new Alert("Coral2 intake motor disconnected", AlertType.kError);
    algaeIntakeDisconnectAlert = new Alert("algae intake motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    coralIntakeDisconnectAlert.set(!inputs.coralIntakeConnected);
    coral2IntakeDisconnectAlert.set(!inputs.coral2IntakeConnected);
    algaeIntakeDisconnectAlert.set(!inputs.algaeIntakeConnected);
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

