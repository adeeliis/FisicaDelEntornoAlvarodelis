import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

BASE_SPEED = 2.0       # velocidad base de las ruedas
HEADING_KP = 0.10      # ganancia proporcional para corregir la dirección

V_MIN = 0.0001         # velocidad mínima para empezar a considerar que empuja contra la pared
V_MAX = 0.0005         # velocidad máxima en la que asumimos que está casi parado

def heading_degrees(sim, sensor_handle) -> float:
    
    #Devuelve la orientación yaw del orientationSensor en grados.
    alpha, beta, gamma = sim.getObjectOrientation(sensor_handle)

    # gamma es la rotación alrededor del eje Z
    return math.degrees(gamma)


def main():
    # Conexión con CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require("sim")

    sim.setStepping(True)
    sim.startSimulation()
    sim.addLog(sim.verbosity_scriptinfos, "Práctica 4 - Redundancia de sensores (main4.py)")

    # Obtener handles
    robot = sim.getObject("/PioneerP3DX")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    orientation_sensor = sim.getObject("/PioneerP3DX/orientationSensor")

    # Ángulo de referencia: orientación inicial
    reference_heading = heading_degrees(sim, orientation_sensor)
    sim.addLog(sim.verbosity_scriptinfos,
               f"Ángulo de referencia inicial: {reference_heading:.2f} grados")

    # Velocidad inicial recta
    sim.setJointTargetVelocity(left_motor, BASE_SPEED)
    sim.setJointTargetVelocity(right_motor, BASE_SPEED)

    sim.addLog(sim.verbosity_scriptinfos, "Entrando en el bucle principal de control.")

    while True:
        # Control de orientación para ir recto
        current_heading = heading_degrees(sim, orientation_sensor)
        error = reference_heading - current_heading

        correction = HEADING_KP * error

        # rueda izquierda fija, rueda derecha corrige
        vel_left = BASE_SPEED
        vel_right = BASE_SPEED + correction

        # para evitar girar en el sitio, no dejamos que la derecha vaya hacia atrás
        if vel_right < 0.0:
            vel_right = 0.0

        sim.setJointTargetVelocity(left_motor, vel_left)
        sim.setJointTargetVelocity(right_motor, vel_right)

        #Detección de colisión frontal por velocidad
        linear_velocity, _ = sim.getObjectVelocity(robot)
        vx, vy = linear_velocity[0], linear_velocity[1]
        speed = math.sqrt(vx * vx + vy * vy)

        if V_MIN <= speed < V_MAX:
            # asumimos que está empotrado contra la pared del fondo
            sim.addLog(sim.verbosity_scriptinfos,
                       f"Colisión detectada (velocidad ≈ {speed:.6f} m/s). Deteniendo robot.")
            sim.setJointTargetVelocity(left_motor, 0.0)
            sim.setJointTargetVelocity(right_motor, 0.0)
            print(f"Colisión detectada. Velocidad lineal ≈ {speed:.6f} m/s.")
            sim.stopSimulation()
            break

        sim.step()


if __name__ == "__main__":
    main()