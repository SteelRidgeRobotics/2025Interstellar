package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    // make sim motors (coral intake bundles the 2 motors together because they work in tandem)
    private static final DCMotor coralIntakeSIM = DCMotor.getKrakenX60(2);
    private static final DCMotor algaeIntakeSIM = DCMotor.getKrakenX60(1);

    private final DCMotorSim intakeSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                coralIntakeSIM, 0.01,Constants.IntakeConstants.GEAR_RATIO),
            coralIntakeSIM);

    private double intakeAppliedVolts = 0.0;
    private boolean intakeIgnoreLimits = false;


    
}
