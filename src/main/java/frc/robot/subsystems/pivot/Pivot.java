package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private TalonFX masterMotor = new TalonFX(frc.robot.Constants.CanIDs.LEFT_PIVOT_TALON);
  private TalonFX followerMotor = new TalonFX(frc.robot.Constants.CanIDs.RIGHT_PIVOT_TALON);
  private MotionMagicDutyCycle motionMagicControl;

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

  public Pivot(PivotIO io) {

    this.io = io;
    setName("Pivot");

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.RotorToSensorRatio = Constants.PivotConstants.GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = Constants.PivotConstants.GAINS;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.MM_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.MM_ACCELERATION;
    config.MotionMagic.MotionMagicJerk = Constants.PivotConstants.MM_CRUISE_VELOCITY * 0.2;

    masterMotor.getConfigurator().apply(config);
    followerMotor.getConfigurator().apply(config);

    Follower followerRequest = new Follower(frc.robot.Constants.CanIDs.LEFT_PIVOT_TALON, true);
    followerMotor.setControl(followerRequest);

    masterMotor.setPosition(0);
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

  @AutoLogOutput(key = "Pivot/State")
  public State getState() {
    return currentState;
  }

  public void setPosition(double setpoint) {

    masterMotor.setControl(motionMagicControl.withPosition(setpoint));
  }
}
