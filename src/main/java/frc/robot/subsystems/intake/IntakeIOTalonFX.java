package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
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
    private final TalonFX topIntakeTalon;
    private final TalonFX bottomIntakeTalon;
    private final CANrange intakeCANrange;

    // make request to edit later
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    // input logging
    private final StatusSignal<AngularVelocity> topIntakeVelocity;
    private final StatusSignal<AngularVelocity> bottomIntakeVelocity;
    private final StatusSignal<Voltage> topIntakeAppliedVoltage;
    private final StatusSignal<Voltage> bottomIntakeAppliedVoltage;
    private final StatusSignal<Current> topIntakeCurrentAmps;
    private final StatusSignal<Current> bottomIntakeCurrentAmps;
    private final StatusSignal<Boolean> coralDetected;

    public IntakeIOTalonFX() {
        // set motor ids
        topIntakeTalon = new TalonFX(Constants.CanIDs.INTAKE_TALON);
        bottomIntakeTalon = new TalonFX(Constants.CanIDs.INTAKE_TALON2);
        intakeCANrange = new CANrange(Constants.CanIDs.INTAKE_CANRANGE);

        var intakeConfig = new TalonFXConfiguration();
        intakeConfig.Slot0.kP = Constants.IntakeConstants.GAINS.kP;
        intakeConfig.Slot0.kI = Constants.IntakeConstants.GAINS.kI;
        intakeConfig.Slot0.kD = Constants.IntakeConstants.GAINS.kD;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.GEAR_RATIO;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT;
        tryUntilOk(5, () -> topIntakeTalon.getConfigurator().apply(intakeConfig));
        tryUntilOk(5, () -> bottomIntakeTalon.getConfigurator().apply(intakeConfig));

        var CANrangeConfig = new CANrangeConfiguration();
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.1;
        tryUntilOk(5, () -> intakeCANrange.getConfigurator().apply(CANrangeConfig));

        // status signal creation
        topIntakeVelocity = topIntakeTalon.getVelocity();
        bottomIntakeVelocity = bottomIntakeTalon.getVelocity();

        topIntakeAppliedVoltage = topIntakeTalon.getMotorVoltage();
        bottomIntakeAppliedVoltage = bottomIntakeTalon.getMotorVoltage();
      
        topIntakeCurrentAmps = topIntakeTalon.getStatorCurrent();
        bottomIntakeCurrentAmps = bottomIntakeTalon.getStatorCurrent();

        coralDetected = intakeCANrange.getIsDetected();

        // determine update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, topIntakeVelocity, topIntakeAppliedVoltage, topIntakeCurrentAmps, coralDetected);
        topIntakeTalon.optimizeBusUtilization();

        BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, bottomIntakeVelocity, bottomIntakeAppliedVoltage, bottomIntakeCurrentAmps, coralDetected);
        bottomIntakeTalon.optimizeBusUtilization();
    }
        //connection thing (may be a hard ctrlc ctrlv)
        final Debouncer topIntakeConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);

        final Debouncer bottomIntakeConnectedDebounce =
        new Debouncer(0.5, Debouncer.DebounceType.kFalling);


    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        var topIntakeStatus = 
            BaseStatusSignal.refreshAll(
            topIntakeAppliedVoltage, 
            topIntakeCurrentAmps,
            topIntakeVelocity 
        );

        var bottomIntakeStatus = 
            BaseStatusSignal.refreshAll(
                bottomIntakeAppliedVoltage,
                bottomIntakeCurrentAmps,
                bottomIntakeVelocity
            );

            inputs.topIntakeConnected = topIntakeConnectedDebounce.calculate(topIntakeStatus.isOK());
            inputs.bottomIntakeConnected = bottomIntakeConnectedDebounce.calculate(bottomIntakeStatus.isOK());
            
            inputs.topIntakeVelocityRadsPerSec = Units.rotationsToRadians(topIntakeVelocity.getValueAsDouble());
            inputs.bottomIntakeVelocityRadsPerSec = Units.rotationsToRadians(bottomIntakeVelocity.getValueAsDouble());
            
            inputs.topIntakeAppliedVoltage = topIntakeAppliedVoltage.getValueAsDouble();
            inputs.bottomIntakeAppliedVoltage = bottomIntakeAppliedVoltage.getValueAsDouble();

            inputs.topIntakeCurrentAmps = topIntakeCurrentAmps.getValueAsDouble();
            inputs.bottomIntakeCurrentAmps = bottomIntakeCurrentAmps.getValueAsDouble();

            inputs.sensorSensed = coralDetected.getValue();
    }
    
    @Override
  public void setIntakeOpenLoop(double output, boolean ignoreLimits) {
    topIntakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
        
    bottomIntakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
      }
  }

