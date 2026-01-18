from math import inf
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

VELOCIDAD_AVANCE = 3.0   # rad/s en las ruedas
DISTANCIA_PARADA = 0.2   # metros: cuando esté a esta distancia o menos, se detiene


def distancia_sensor(sim, sensor_handle):
    #Devuelve la distancia medida por el sensor
    hit, distance, detected_point, detected_object, normal_vector = sim.readProximitySensor(sensor_handle)
    return distance if hit else inf


def obtener_handles_robot(sim):
    #Obtiene los motores y sensores frontales del PioneerP3DX
    sim.addLog(sim.verbosity_scriptinfos, "Obteniendo handles del PioneerP3DX...")

    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")

    front_left_sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor[3]")
    front_right_sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor[4]")

    return right_motor, left_motor, front_left_sensor, front_right_sensor


def main():
    # Conectar con CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require("sim")

    sim.setStepping(True)
    sim.startSimulation()
    sim.addLog(sim.verbosity_scriptinfos, "Simulación iniciada desde main2.py")

    # Obtener motores y sensores
    right_motor, left_motor, s_front_left, s_front_right = obtener_handles_robot(sim)

    # Lectura inicial para activar los sensores
    _ = distancia_sensor(sim, s_front_left)
    _ = distancia_sensor(sim, s_front_right)

    # Avanzar recto hasta estar a DISTANCIA_PARADA o menos
    sim.addLog(sim.verbosity_scriptinfos, "Comenzando avance en línea recta hacia el cilindro.")
    print("El robot comienza a avanzar hacia el cilindro...")

    while True:
        d_izq = distancia_sensor(sim, s_front_left)
        d_der = distancia_sensor(sim, s_front_right)
        distancia_min = min(d_izq, d_der)

        if distancia_min <= DISTANCIA_PARADA:
            # Si está lo bastante cerca frena en seco y sale del bucle
            sim.setJointTargetVelocity(right_motor, 0.0)
            sim.setJointTargetVelocity(left_motor, 0.0)

            mensaje = f"Robot detenido a {distancia_min:.3f} m del cilindro."
            sim.addLog(sim.verbosity_scriptinfos, mensaje)
            print(mensaje)

            break
        else:
            # Si Ttodavía esta lejos sigue avanzando recto
            sim.setJointTargetVelocity(right_motor, VELOCIDAD_AVANCE)
            sim.setJointTargetVelocity(left_motor, VELOCIDAD_AVANCE)
            sim.addLog(
                sim.verbosity_scriptinfos,
                f"Avanzando. Distancia actual al cilindro: {distancia_min:.3f} m",
            )

        sim.step()

if __name__ == "__main__":
    main()