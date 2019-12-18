package frc.robot.subsystems.superstructure.controller

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N2
import frc.robot.subsystems.superstructure.Elevator
import frc.robot.subsystems.superstructure.Superstructure
import org.ejml.simple.SimpleMatrix
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.team5940.pantry.lib.WantedState

class ElevatorSSTest: FalconCommand(Superstructure, Elevator) {

    private val constraints = TrapezoidProfile.Constraints(8.inches.inMeters(), 8.inches.inMeters())

    override fun isFinished(): Boolean {
        return false
    }

    override fun execute() {
        // let's take the elevator up of it's down and down if it's up
        val targetHeight = 40.inches

        ElevatorController.enable()

        val state = Elevator.currentState

        val profile = TrapezoidProfile(
                constraints, TrapezoidProfile.State(targetHeight.inMeters(), 0.0),
                TrapezoidProfile.State(state.position.value, state.velocity.value)
        )

        val target = profile.calculate(0.040)
//        ElevatorController.nextR = MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(target.position, target.velocity)
        ElevatorController.nextR = MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(targetHeight.inMeters(), 0.0)

        val y = ElevatorController.plant.calculateY(
                MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(state.position.value, state.velocity.value),
                ElevatorController.u)

        ElevatorController.correct(y)

//        ElevatorController.correct(MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(state.position.value, state.velocity.value))
        ElevatorController.predict(0.020)

        var voltage = ElevatorController.getU(0)

        println("error ${ElevatorController.error.get(0, 0).meters.inInches()} voltage: $voltage")

        voltage = voltage.coerceIn(-8.0, 8.0)
        Elevator.wantsLowGear = false
        Elevator.wantedState = WantedState.Voltage(voltage.volts)
    }

}