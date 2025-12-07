package frc.robot.subsystems.pivot;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class PivotIOTalonFX implements PivotIO {

  private final TalonFX mainTalon;
  private final TalonFX followerTalon;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(
          0,
          Constants.PivotConstants.MM_CRUISE_VELOCITY,
          Constants.PivotConstants.MM_ACCELERATION,
          0);

  private final StatusSignal<Angle> mainPosition;
  private final StatusSignal<AngularVelocity> mainVelocity;
  private final StatusSignal<Voltage> mainAppliedVolts;
  private final StatusSignal<Current> mainStatorCurrent;

  private final Debouncer mainTalonConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerTalonConnectedDebounce = new Debouncer(0.5);

  public PivotIOTalonFX() {
    mainTalon = new TalonFX(Constants.CanIDs.LEFT_PIVOT_TALON);
    followerTalon = new TalonFX(Constants.CanIDs.RIGHT_PIVOT_TALON);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0 = Constants.PivotConstants.GAINS;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Feedback.SensorToMechanismRatio = Constants.PivotConstants.GEAR_RATIO;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.MM_ACCELERATION;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.MM_CRUISE_VELOCITY;

    tryUntilOk(5, () -> mainTalon.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> followerTalon.getConfigurator().apply(motorConfig, 0.25));

    mainPosition = mainTalon.getPosition();
    mainVelocity = mainTalon.getVelocity();
    mainAppliedVolts = mainTalon.getMotorVoltage();
    mainStatorCurrent = mainTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, mainVelocity, mainAppliedVolts, mainStatorCurrent);

    ParentDevice.optimizeBusUtilizationForAll(mainTalon, followerTalon);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    var talonStatus =
        BaseStatusSignal.refreshAll(
            mainPosition, mainVelocity, mainAppliedVolts, mainStatorCurrent);

    inputs.pivotConnected = mainTalonConnectedDebounce.calculate(talonStatus.isOK());
    inputs.pivotConnected = followerTalonConnectedDebounce.calculate(followerTalon.isConnected());
    inputs.positionRads = Units.rotationsToRadians(mainPosition.getValueAsDouble());
    inputs.velocityRads = Units.rotationsToRadians(mainVelocity.getValueAsDouble());
    inputs.pivotAppliedVolts = mainAppliedVolts.getValueAsDouble();
    inputs.statorCurrent = mainStatorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    mainTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    mainTalon.setControl(positionRequest.withPosition(rotation.getRotations()));
  }
}
