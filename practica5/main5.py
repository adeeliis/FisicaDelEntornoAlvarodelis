from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2 as cv
import numpy as np

# Estados de la máquina
STATE_SPIN = 0       # girar hasta ver algo rojo
STATE_ALIGN = 1      # ajustar orientación para centrar el objeto
STATE_APPROACH = 2   # avanzar hacia el objeto
STATE_STOP = 3       # parado

# Parámetros de movimiento
SPIN_SPEED = 0.4         # velocidad de giro
FORWARD_SPEED = 2.0      # velocidad al avanzar
CENTER_TOLERANCE = 0.05  # qué parte del ancho consideramos


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')

    sim.setStepping(True)
    sim.startSimulation()

    sim.addLog(sim.verbosity_scriptinfos, "Práctica 5 - Detección de colores")

    right_motor = sim.getObject("/PioneerP3DX/rightMotor")
    left_motor = sim.getObject("/PioneerP3DX/leftMotor")
    camera = sim.getObject("/PioneerP3DX/camera")

    # Estado inicial
    state = STATE_SPIN
    center_x = None

    # Robot parado al principio
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)

    cv.namedWindow("Mask", cv.WINDOW_NORMAL)

    sim.addLog(sim.verbosity_scriptinfos, "Starting main loop.")

    while True:
        #Captura y procesado de imagen
        raw_image, resolution = sim.getVisionSensorImg(camera)

        image = np.frombuffer(raw_image, dtype=np.uint8)
        image = image.reshape([resolution[1], resolution[0], 3])
        image = cv.cvtColor(image, cv.COLOR_RGB2HSV)
        image = np.rot90(image, 2)
        image = np.fliplr(image)

        # Umbrales para el color rojo en HSV
        lower_red_1 = np.array([0, 100, 100])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([160, 100, 100])
        upper_red_2 = np.array([180, 255, 255])

        mask_1 = cv.inRange(image, lower_red_1, upper_red_1)
        mask_2 = cv.inRange(image, lower_red_2, upper_red_2)
        mask = cv.bitwise_or(mask_1, mask_2)

        # Índices de columna al 10% y 90% del ancho de la imagen
        height, width = mask.shape[:2]
        x10 = int(width * 0.10)
        x90 = int(width * 0.90)

        # Cálculo del centroide del área roja
        moments = cv.moments(mask)
        has_red = moments["m00"] > 0

        if has_red:
            center_x = int(moments["m10"] / moments["m00"])
            center_y = int(moments["m01"] / moments["m00"])
        else:
            center_x = None

        # Detección de presencia del objeto en las columnas del 10% y 90%
        hit_x10 = False
        hit_x90 = False
        if 0 <= x10 < width:
            hit_x10 = np.any(mask[:, x10] > 0)
        if 0 <= x90 < width:
            hit_x90 = np.any(mask[:, x90] > 0)

        # Mostrar solo la máscara
        display_image = mask.copy()
        cv.resizeWindow("Mask", resolution[0], resolution[1])
        cv.imshow("Mask", display_image)
        key = cv.waitKey(5)
        if key == 27:  # ESC para salir manualmente
            sim.addLog(sim.verbosity_scriptinfos, "ESC pulsado, saliendo de la práctica 5.")
            break

        #Máquina de estados
        if state == STATE_SPIN:
            # Girar sobre sí mismo hasta que aparezca algo rojo en la imagen
            sim.setJointTargetVelocity(left_motor, -SPIN_SPEED)
            sim.setJointTargetVelocity(right_motor, SPIN_SPEED)

            if has_red and center_x is not None:
                sim.addLog(sim.verbosity_scriptinfos, "Objeto rojo detectado. Pasando a ALIGN.")
                state = STATE_ALIGN

        elif state == STATE_ALIGN:
            # Ajustar la orientación hasta que el objeto esté centrado horizontalmente
            if not has_red or center_x is None:
                # Hemos perdido el objeto -> volvemos a girar
                sim.addLog(sim.verbosity_scriptinfos, "Objeto perdido. Volviendo a SPIN.")
                state = STATE_SPIN
            else:
                # Error respecto al centro de la imagen
                error_x = center_x - (width / 2.0)
                # Normalizamos el error a [-1, 1] aproximado
                norm_error = error_x / (width / 2.0)

                if abs(norm_error) < CENTER_TOLERANCE:
                    # Está suficientemente centrado -> avanzar
                    sim.addLog(sim.verbosity_scriptinfos, "Objeto centrado. Pasando a APPROACH.")
                    state = STATE_APPROACH
                    sim.setJointTargetVelocity(left_motor, FORWARD_SPEED)
                    sim.setJointTargetVelocity(right_motor, FORWARD_SPEED)
                else:
                    # girar un poco hacia el lado donde está el objeto
                    if norm_error > 0:
                        # objeto a la derecha -> girar a la derecha
                        sim.setJointTargetVelocity(left_motor, SPIN_SPEED)
                        sim.setJointTargetVelocity(right_motor, -SPIN_SPEED)
                    else:
                        # objeto a la izquierda -> girar a la izquierda
                        sim.setJointTargetVelocity(left_motor, -SPIN_SPEED)
                        sim.setJointTargetVelocity(right_motor, SPIN_SPEED)

        elif state == STATE_APPROACH:
            # Avanzar hacia el objeto
            if not has_red:
                # Si lo perdemos, dejamos de avanzar y volvemos a buscar
                sim.addLog(sim.verbosity_scriptinfos, "Objeto perdido durante el avance. Volviendo a SPIN.")
                sim.setJointTargetVelocity(left_motor, 0.0)
                sim.setJointTargetVelocity(right_motor, 0.0)
                state = STATE_SPIN
            else:
                # Mantener avance recto (simple)
                sim.setJointTargetVelocity(left_motor, FORWARD_SPEED)
                sim.setJointTargetVelocity(right_motor, FORWARD_SPEED)

                # Si el objeto está en el 10% y 90% del ancho simultáneamente,
                # significa que está muy cerca -> detenemos el robot
                if hit_x10 and hit_x90:
                    sim.addLog(sim.verbosity_scriptinfos, "Objeto muy cercano. Parando.")
                    state = STATE_STOP

        elif state == STATE_STOP:
            sim.setJointTargetVelocity(left_motor, 0.0)
            sim.setJointTargetVelocity(right_motor, 0.0)
            sim.addLog(sim.verbosity_scriptinfos, "Robot detenido. Fin.")
            sim.stopSimulation()
            break

        else:
            # Estado no esperado -> seguridad
            sim.addLog(sim.verbosity_scripterrors, "Estado desconocido en la FSM. Deteniendo.")
            sim.setJointTargetVelocity(left_motor, 0.0)
            sim.setJointTargetVelocity(right_motor, 0.0)
            sim.stopSimulation()
            break

        sim.step()

    cv.destroyAllWindows()
    sim.stopSimulation()


if __name__ == "__main__":
    main()