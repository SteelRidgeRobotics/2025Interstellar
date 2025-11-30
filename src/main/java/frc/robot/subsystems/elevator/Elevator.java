package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  public enum State {
    IDLE(null),
    DEFAULT(ElevatorConstants.DEFAULT_POSITION),
    L1(ElevatorConstants.L1_SCORE_POSITION),
    L2(ElevatorConstants.L2_SCORE_POSITION),
    L3(ElevatorConstants.L3_SCORE_POSITION),
    L2_ALGAE(ElevatorConstants.L2_ALGAE_POSITION),
    L3_ALGAE(ElevatorConstants.L3_ALGAE_POSITION),
    PROCESSOR(ElevatorConstants.PROCESSOR_SCORE_POSITION),
    MAX(ElevatorConstants.ELEVATOR_MAX);

    public final Double position;

    State(Double position) {
      this.position = position;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private State currentState = State.DEFAULT;

  private SysIdRoutine sysID;

  private boolean atSetpoint = true;
  private final Debouncer atSetpointDebounce;

  public Elevator(ElevatorIO io) {
    this.io = io;
    setName("Elevator");

    atSetpointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

    sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(12),
                Seconds.of(10.0),
                state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.setOpenLoop(output.in(Volts)), log -> {}, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(State state) {
    currentState = state;
    io.setPosition(Optional.ofNullable(currentState.position));
  }

  @AutoLogOutput(key = "Elevator/State")
  public State getState() {
    return currentState;
  }

  @AutoLogOutput(key = "Elevator/At Setpoint")
  public boolean isAtSetpoint() {
    if (getState() == State.IDLE) return false;
    atSetpoint =
        Math.abs(Units.radiansToRotations(inputs.positionRads) - currentState.position)
            <= ElevatorConstants.SETPOINT_TOLERANCE;

    return atSetpoint;
  }

  public Command stop() {
    return new InstantCommand(() -> io.setOpenLoop(0), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysID.quasistatic(direction).andThen(stop());
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysID.dynamic(direction).andThen(stop());
  }

  public double getHeight() {
    return (Units.radiansToRotations(inputs.positionRads) / ElevatorConstants.GEAR_RATIO)
        * (2 * Math.PI * 0.508);
  }

  public Pose3d[] getComponentPoses() {
    double position = Units.radiansToRotations(inputs.positionRads);
    return new Pose3d[] {
      new Pose3d(0, 0, position * (0.6985 / 6.096924), new Rotation3d()),
      new Pose3d(0, 0, position * 2 * (0.6985 / 6.096924), new Rotation3d())
    };
  }
}
