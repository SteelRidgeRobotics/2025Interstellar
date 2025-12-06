package frc.robot.subsystems.elevator;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO{
    private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
    private final DCMotorSim elevatorSim;

    private boolean closedLoop = true;

    private final ProfiledPIDController elevatorController =
        new ProfiledPIDController(
            ElevatorConstants.GAINS.kP / (2 * Math.PI),
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(ElevatorConstants.MM_CRUISE_VELOCITY), 
                Units.rotationsToRadians(ElevatorConstants.MM_UPWARDS_ACCELERATION)
            )
        );

    private double elevatorAppiedVolts;
    
    public ElevatorIOSim() {
        elevatorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(ELEVATOR_GEARBOX, 0.01, ElevatorConstants.GEAR_RATIO),
                ELEVATOR_GEARBOX
            );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (closedLoop) {
            elevatorAppiedVolts = elevatorController.calculate(elevatorSim.getAngularPositionRad());
        } else {
            elevatorController.reset(
                elevatorSim.getAngularPositionRad(), elevatorSim.getAngularVelocityRadPerSec()
            );
        }

        elevatorSim.setInputVoltage(MathUtil.clamp(elevatorAppiedVolts, -12, 12));
        elevatorSim.update(0.02);

        inputs.elevatorConnected = true;
        inputs.positionRads = elevatorSim.getAngularPositionRad();
        inputs.velocityRads = elevatorSim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = elevatorAppiedVolts;
        inputs.statorCurrent = Math.abs(elevatorSim.getCurrentDrawAmps());
    }

    @Override
    public void setOpenLoop(double output) {
        closedLoop = false;
        elevatorAppiedVolts = output;
    }

    @Override
    public void setPosition(Optional<Double> position) {
        if (position.isEmpty()) {
            setOpenLoop(0);
            return;
        }

        closedLoop = true;

        double pos = Units.rotationsToRadians(position.get());
        elevatorController.setGoal(pos);

        if (pos > elevatorSim.getAngularPositionRad()) {
            elevatorController.setConstraints(new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(ElevatorConstants.MM_CRUISE_VELOCITY),
                Units.rotationsToRadians(ElevatorConstants.MM_BRAKE_ACCELERATION)
            ));
        } else {
            elevatorController.setConstraints(
                new TrapezoidProfile.Constraints(
                    Units.rotationsToRadians(ElevatorConstants.MM_CRUISE_VELOCITY),
                    Units.rotationsToRadians(ElevatorConstants.MM_DOWNWARD_ACCELERATION)
                )
            );
        }
    }
}
