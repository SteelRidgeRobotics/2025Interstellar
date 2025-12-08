package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
// import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  // private TalonFX masterMotor = new TalonFX(frc.robot.Constants.CanIDs.LEFT_PIVOT_TALON);
  // private TalonFX followerMotor = new TalonFX(frc.robot.Constants.CanIDs.RIGHT_PIVOT_TALON);
  // private MotionMagicDutyCycle motionMagicControl;

  public enum State {
    DEFAULT(PivotConstants.START_ANGLE),
    SCORING(PivotConstants.SCORING_ANGLE),
    INTAKE(PivotConstants.GROUND_INTAKE_ANGLE);

    private final Rotation2d rotation;

    State(double rotations) {
      this.rotation = Rotation2d.fromRotations(rotations);
    }
  }

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private State currentState = State.DEFAULT;

  private final SysIdRoutine sysIdRoutine;

  private final Debouncer atSetPointDebounce;

  public Pivot(PivotIO io) {

    this.io = io;
    setName("Pivot");

    atSetPointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(12),
                Seconds.of(10.0),
                state -> SignalLogger.writeString("SysIdPivot_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> io.setOpenLoop(output.in(Volts)), log -> {}, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }

  public void setState(State state) {
    currentState = state;
    io.setPosition(currentState.rotation);
  }

  @AutoLogOutput(key = "Pivot/At Setpoint")
  public boolean isAtSetpoint() {
    return atSetPointDebounce.calculate(
        Math.abs(inputs.pivotAbsolutePosition.minus(currentState.rotation).getRotations())
            <= Constants.PivotConstants.SETPOINT_TOLERANCE);
  }

  @AutoLogOutput(key = "Pivot/State")
  public State getState() {
    return currentState;
  }

  public Command stop() {
    return runOnce(() -> io.setOpenLoop(0.0));
  }

  public void sysIdQuasistatic(SysIdRoutine.Direction direction) {
    sysIdRoutine.quasistatic(direction).andThen(this.stop());
  }

  public void sysIdDynamic(SysIdRoutine.Direction direction) {
    sysIdRoutine.quasistatic(direction).andThen(this.stop());
  }

  /*public void setPosition(double setpoint) {

    masterMotor.setControl(motionMagicControl.withPosition(setpoint));
  } */
}
