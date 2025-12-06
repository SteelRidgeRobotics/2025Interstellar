package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX mainTalon;
  private final TalonFX followerTalon;

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(
          0,
          Constants.ElevatorConstants.MM_CRUISE_VELOCITY,
          Constants.ElevatorConstants.MM_UPWARDS_ACCELERATION,
          Constants.ElevatorConstants.MM_JERK);

  private final DynamicMotionMagicVoltage brakeRequest =
      new DynamicMotionMagicVoltage(
          0, ElevatorConstants.MM_CRUISE_VELOCITY, ElevatorConstants.MM_BRAKE_ACCELERATION, 0);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> mainPosition;
  private final StatusSignal<AngularVelocity> mainVelocity;
  private final StatusSignal<Voltage> mainAppliedVolts;
  private final StatusSignal<Current> mainStatorCurrent;

  private final Debouncer mainTalonConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerTalonConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ElevatorIOTalonFX() {
    mainTalon = new TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON);
    followerTalon = new TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON);

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(ElevatorConstants.GAINS)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEAR_RATIO))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(ElevatorConstants.MM_DOWNWARD_ACCELERATION)
                    .withMotionMagicCruiseVelocity(ElevatorConstants.MM_CRUISE_VELOCITY));

    tryUntilOk(5, () -> mainTalon.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> followerTalon.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> mainTalon.setPosition(ElevatorConstants.DEFAULT_POSITION));

    mainPosition = mainTalon.getPosition();
    mainVelocity = mainTalon.getVelocity();
    mainAppliedVolts = mainTalon.getMotorVoltage();
    mainStatorCurrent = mainTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(250, mainPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, mainVelocity, mainAppliedVolts, mainStatorCurrent);

    ParentDevice.optimizeBusUtilizationForAll(mainTalon, followerTalon);

    mainTalon.setControl(brakeRequest);
    // followerTalon.setControl(new Follower(mainTalon.getDeviceID(), true));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var talonStatus =
        BaseStatusSignal.refreshAll(
            mainPosition, mainVelocity, mainAppliedVolts, mainStatorCurrent);

    inputs.elevatorConnected =
        mainTalonConnectedDebounce.calculate(talonStatus.isOK())
            && followerTalonConnectedDebounce.calculate(followerTalon.isConnected());

    inputs.positionRads = Units.rotationsToRadians(mainPosition.getValueAsDouble());
    inputs.velocityRads = Units.rotationsToRadians(mainVelocity.getValueAsDouble());
    inputs.appliedVolts = mainAppliedVolts.getValueAsDouble();
    inputs.statorCurrent = mainStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    mainTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(double position) {

      if (mainTalon.getPosition().getValue().in(Rotations) < position) {
        positionRequest.Acceleration = ElevatorConstants.MM_UPWARDS_ACCELERATION;
      } else {
        positionRequest.Acceleration = ElevatorConstants.MM_DOWNWARD_ACCELERATION;
      }

      positionRequest.Position = position;
      mainTalon.setControl(positionRequest);
  }

}
