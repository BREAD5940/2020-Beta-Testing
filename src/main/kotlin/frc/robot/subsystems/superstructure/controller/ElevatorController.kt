package frc.robot.subsystems.superstructure.controller

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpilibj.system.LinearSystemLoop
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N2
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches

val plant: LinearSystem<N2, N1, N1> = LinearSystem.createElevatorSystem(
        DCMotor.getVex775Pro(4),
        8.0,
        0.75.inches.inMeters(),
        14.67,
        8.0
)

const val kDt = 0.020

object ElevatorController : LinearSystemLoop<N2, N1, N1>(
        Nat.N2(), Nat.N1(), Nat.N1(),
        plant,
        LinearQuadraticRegulator(
                Nat.N2(), Nat.N1(), plant,
                MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(0.08, 1.0),
                MatBuilder<N1, N1>(Nat.N1(), Nat.N1()).fill(12.0),
                kDt
        ),
        KalmanFilter(
                Nat.N2(), Nat.N1(), Nat.N1(), plant,
                MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(0.05, 200.0),
                MatBuilder<N1, N1>(Nat.N1(), Nat.N1()).fill(0.0001),
                kDt
        )
)