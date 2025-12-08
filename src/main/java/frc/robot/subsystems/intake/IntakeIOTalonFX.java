package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {

  // make motors
  private final TalonFX coral2IntakeTalon;
  private final TalonFX coralIntakeTalon;
  private final TalonFX algaeIntakeTalon;
  private final CANrange intakeCANrange;

  // make request to edit later
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // input logging
  private final StatusSignal<AngularVelocity> coral2IntakeVelocity;
  private final StatusSignal<AngularVelocity> coralIntakeVelocity;
  private final StatusSignal<AngularVelocity> algaeIntakeVelocity;

  private final StatusSignal<Voltage> coral2IntakeAppliedVoltage;
  private final StatusSignal<Voltage> coralIntakeAppliedVoltage;
  private final StatusSignal<Voltage> algaeIntakeAppliedVoltage;

  private final StatusSignal<Current> coral2IntakeCurrentAmps;
  private final StatusSignal<Current> coralIntakeCurrentAmps;
  private final StatusSignal<Current> algaeIntakeCurrentAmps;

  private final StatusSignal<Boolean> coralDetected;

  public IntakeIOTalonFX() {
    // set motor ids
    coralIntakeTalon = new TalonFX(Constants.CanIDs.INTAKE_TALON);
    coral2IntakeTalon = new TalonFX(Constants.CanIDs.INTAKE_TALON2);
    algaeIntakeTalon = new TalonFX(Constants.CanIDs.ALGAE_TALON);
    intakeCANrange = new CANrange(Constants.CanIDs.INTAKE_CANRANGE);

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.Slot0.kP = Constants.IntakeConstants.GAINS.kP;
    intakeConfig.Slot0.kI = Constants.IntakeConstants.GAINS.kI;
    intakeConfig.Slot0.kD = Constants.IntakeConstants.GAINS.kD;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.GEAR_RATIO;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT;
    tryUntilOk(5, () -> coral2IntakeTalon.getConfigurator().apply(intakeConfig));
    tryUntilOk(5, () -> coralIntakeTalon.getConfigurator().apply(intakeConfig));
    tryUntilOk(5, () -> algaeIntakeTalon.getConfigurator().apply(intakeConfig));

    var CANrangeConfig = new CANrangeConfiguration();
    CANrangeConfig.ProximityParams.ProximityThreshold = 0.1;
    tryUntilOk(5, () -> intakeCANrange.getConfigurator().apply(CANrangeConfig));

    // status signal creation
    coral2IntakeVelocity = coral2IntakeTalon.getVelocity();
    coralIntakeVelocity = coralIntakeTalon.getVelocity();
    algaeIntakeVelocity = algaeIntakeTalon.getVelocity();

    coral2IntakeAppliedVoltage = coral2IntakeTalon.getMotorVoltage();
    coralIntakeAppliedVoltage = coralIntakeTalon.getMotorVoltage();
    algaeIntakeAppliedVoltage = coralIntakeTalon.getMotorVoltage();

    coral2IntakeCurrentAmps = coral2IntakeTalon.getStatorCurrent();
    coralIntakeCurrentAmps = coralIntakeTalon.getStatorCurrent();
    algaeIntakeCurrentAmps = coralIntakeTalon.getStatorCurrent();

    coralDetected = intakeCANrange.getIsDetected();

    // determine update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        coral2IntakeVelocity,
        coral2IntakeAppliedVoltage,
        coral2IntakeCurrentAmps,
        coralIntakeVelocity,
        coralIntakeAppliedVoltage,
        coralIntakeCurrentAmps,
        algaeIntakeVelocity,
        algaeIntakeAppliedVoltage,
        algaeIntakeCurrentAmps,
        coralDetected);
    ParentDevice.optimizeBusUtilizationForAll(
        coralIntakeTalon, coral2IntakeTalon, algaeIntakeTalon);
  }

  // connection thing (may be a hard ctrlc ctrlv)
  final Debouncer coral2IntakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  final Debouncer coralIntakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  final Debouncer algaeIntakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var coral2IntakeStatus =
        BaseStatusSignal.refreshAll(
            coral2IntakeAppliedVoltage, coral2IntakeCurrentAmps, coral2IntakeVelocity);

    var coralIntakeStatus =
        BaseStatusSignal.refreshAll(
            coralIntakeAppliedVoltage, coralIntakeCurrentAmps, coralIntakeVelocity);

    var algaeIntakeStatus =
        BaseStatusSignal.refreshAll(
            algaeIntakeAppliedVoltage, algaeIntakeCurrentAmps, algaeIntakeVelocity);

    inputs.coral2IntakeConnected =
        coral2IntakeConnectedDebounce.calculate(coral2IntakeStatus.isOK());
    inputs.coralIntakeConnected = coralIntakeConnectedDebounce.calculate(coralIntakeStatus.isOK());
    inputs.coral2IntakeConnected = algaeIntakeConnectedDebounce.calculate(algaeIntakeStatus.isOK());

    inputs.coral2IntakeVelocityRadsPerSec =
        Units.rotationsToRadians(coral2IntakeVelocity.getValueAsDouble());
    inputs.coralIntakeVelocityRadsPerSec =
        Units.rotationsToRadians(coralIntakeVelocity.getValueAsDouble());

    inputs.coral2IntakeAppliedVoltage = coral2IntakeAppliedVoltage.getValueAsDouble();
    inputs.coralIntakeAppliedVoltage = coralIntakeAppliedVoltage.getValueAsDouble();

    inputs.coral2IntakeCurrentAmps = coral2IntakeCurrentAmps.getValueAsDouble();
    inputs.coralIntakeCurrentAmps = coralIntakeCurrentAmps.getValueAsDouble();

    inputs.sensorSensed = coralDetected.getValue();
  }

  @Override
  public void setCoralOpenLoop(double output, boolean ignoreLimits) {
    coral2IntakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
    coralIntakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
  }

  @Override
  public void setAlgaeOpenLoop(double output, boolean ignoreLimits) {
    algaeIntakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
  }
}
