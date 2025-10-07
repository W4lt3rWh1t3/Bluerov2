from pymavlink import mavutil
import time
import math



def arm(master):
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)


def disarm(master):
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    
#set_position_target
# Funzione per inviare comando diretto al servo (motore)
def set_servo_pwm(master, servo_n, pwm_us):
    """
    Comanda direttamente un'uscita del Pixhawk.
    servo_n: numero dell'uscita (1-8 per MAIN, 9-14 per AUX)
    pwm_us: valore PWM in microsecondi (tipicamente 1100-1900)
    """

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Comando diretto al servo
        0,            # conferma
        servo_n,      # numero del servo (es. 1 per MAIN1)
        pwm_us,       # valore PWM
        0, 0, 0, 0, 0 # parametri inutilizzati
    )
    print(f"Servo {servo_n} -> PWM {pwm_us} us")


def set_multiple_servos_pwm(master, servo_pwm_list):
    """
    Comanda più uscite del Pixhawk contemporaneamente.
    servo_pwm_list: lista di tuple (servo_n, pwm_us)
    """
    for servo_n, pwm_us in servo_pwm_list:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # conferma
            servo_n,      # numero del servo (es. 1 per MAIN1)
            pwm_us,       # valore PWM
            0, 0, 0, 0, 0 # parametri inutilizzati
        )


def set_param(master):
    """
    Imposta più parametri SERVOx_FUNCTION a 0 (Disabled) e ne conferma l'impostazione.
    """

    SERVO_PARAMS = [
        'SERVO1_FUNCTION', 
        'SERVO2_FUNCTION', 
        'SERVO3_FUNCTION', 
        'SERVO4_FUNCTION', 
        'SERVO5_FUNCTION', 
        'SERVO6_FUNCTION', 
        'SERVO7_FUNCTION', 
        'SERVO8_FUNCTION'
    ]
    
    value = 0  # 0 = Disabled
    
    for param in SERVO_PARAMS:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

        # Attesa della conferma (PARAM_VALUE)
        while True:
            message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if message is None:
                print(f"Nessuna conferma ricevuta per {param}")
                break
            if message.param_id.strip('\x00') == param:
                actual = message.param_value
                print(f"Confermato {param} = {actual}")
                break


def forwardMovement(master, duration, power):

    deltaPower = abs(power - 1500)

    motori_attivi = [
        (1, 1500 + deltaPower),  # MAIN1
        (2, 1500 + deltaPower),  # MAIN2
        (3, 1500 + deltaPower),  # MAIN3
        (4, 1500 + deltaPower)   # MAIN4
    ]
    set_multiple_servos_pwm(master, motori_attivi)
    
    time.sleep(duration)

    motori_off = [(1, 1500), (2, 1500), (3, 1500), (4, 1500)]
    set_multiple_servos_pwm(master, motori_off)



def yawMovement(master, degrees, power=1600, yaw_tolerance=5, timeout_sec=10):
    """
    Ruota il ROV di un certo numero di gradi usando il valore ATTITUDE.
    Positivo = orario (destra), negativo = antiorario (sinistra).
    """
    print(f"Rotazione richiesta: {degrees:.1f}°")

    # Riceve primo messaggio ATTITUDE
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not msg:
        print("Errore: nessun dato ATTITUDE ricevuto.")
        return

    initial_yaw = math.degrees(msg.yaw) % 360
    target_yaw = (initial_yaw - degrees) % 360
    print(f"Yaw iniziale: {initial_yaw:.2f}°, target: {target_yaw:.2f}°")

    start_time = time.time()
    direction = 1 if degrees > 0 else -1

    # Imposta potenza in base alla direzione
    pwm_delta = abs(power - 1500)
    motor_1 = 1500 + direction * pwm_delta
    motor_2 = 1500 - direction * pwm_delta
    motor_3 = 1500 + direction * pwm_delta
    motor_4 = 1500 - direction * pwm_delta

    while time.time() - start_time < timeout_sec:
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if not msg:
            continue

        current_yaw = math.degrees(msg.yaw) % 360
        delta = (target_yaw - current_yaw + 180) % 360 - 180

        print(f"Yaw attuale: {current_yaw:.2f}°, delta: {delta:.2f}°")

        if abs(delta) <= yaw_tolerance:
            break

        motori_yaw = [
            (1, motor_1),
            (2, motor_2),
            (3, motor_3),
            (4, motor_4)
        ]
        set_multiple_servos_pwm(master, motori_yaw)
        time.sleep(0.1)

    # Ferma i motori (valore neutro)
    set_multiple_servos_pwm(master, [(1, 1500), (2, 1500), (3, 1500), (4, 1500)])
    print("Rotazione completata")



def throttleMovement(master, target_depth, timeout=15, tolerance=0.1, correction_pwm=100):
    """
    Porta il ROV alla profondità desiderata usando i motori verticali (servo 5 e 6).
    - target_depth: profondità in metri
    - timeout: tempo massimo per raggiungerla
    - tolerance: margine accettabile in metri
    - correction_pwm: variazione PWM per correggere la profondità
    """
    print(f"[INFO] Target profondità: {target_depth:.5f} m")

    # Attendi primo messaggio di profondità
    depth_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not depth_msg:
        print("[ERRORE] Nessun dato di profondità ricevuto.")
        return

    start_time = time.time()

    while time.time() - start_time < timeout:
        depth_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.5)
        if not depth_msg:
            continue

        # Calcolo profondità attuale in metri (approssimazione semplice)
        current_pressure = depth_msg.press_abs  # in mbar
        current_depth = (current_pressure - 1013.25) / 100.0  # ≈ profondità in m

        print(f"[INFO] Profondità attuale: {current_depth:.5f} m")

        delta = target_depth - current_depth

        if abs(delta) <= tolerance:
            print("[INFO] Profondità raggiunta.")
            break

        # Correzione PWM
        pwm = 1500
        if delta > 0:
            pwm -= correction_pwm  # Scende
        else:
            pwm += correction_pwm  # Sale

        # Applica PWM a motori verticali
        motori_verticali = [(5, pwm), (6, 3000 - pwm), (7, 3000 - pwm), (8, pwm)]
        set_multiple_servos_pwm(master, motori_verticali)
 
        time.sleep(0.2)

    # Ferma i motori verticali
    set_multiple_servos_pwm(master, [(5, 1500), (6, 1500), (7, 1500), (8, 1500)])
    print("[INFO] Motori verticali fermati.")



def pitchMovement(master, target_pitch_deg, timeout=15, tolerance=2.0, correction_pwm=100):
    """
    Porta il ROV al pitch desiderato usando i motori verticali o correttivi.
    - target_pitch_deg: angolo in gradi (positivo = nose up, negativo = nose down)
    - timeout: tempo massimo per raggiungere il pitch
    - tolerance: margine di errore in gradi
    - correction_pwm: forza di correzione PWM
    """
    print(f"[INFO] Target pitch: {target_pitch_deg:.2f}°")

    start_time = time.time()

    while time.time() - start_time < timeout:
        # Leggi orientamento
        attitude_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if not attitude_msg:
            continue

        # Converti pitch in gradi
        current_pitch = math.degrees(attitude_msg.pitch)
        delta = target_pitch_deg - current_pitch

        print(f"[INFO] Pitch attuale: {current_pitch:.2f}° | Delta: {delta:.2f}°")

        if abs(delta) <= tolerance:
            print("[INFO] Pitch raggiunto.")
            break

        # PWM correttivo
        # Nota: aggiusta i motori in base alla tua configurazione fisica!
        if delta > 0:
            # Serve nose up → più spinta dietro
            pwm_front = 1500 - correction_pwm
            pwm_back  = 1500 + correction_pwm
        else:
            # Serve nose down → più spinta davanti
            pwm_front = 1500 + correction_pwm
            pwm_back  = 1500 - correction_pwm

        # Esempio: motori 5 e 6 = anteriori, 7 e 8 = posteriori
        motori_pitch = [
            (5, pwm_back),
            (6, pwm_front),
            (7, pwm_back),
            (8, pwm_front)
        ]
        set_multiple_servos_pwm(master, motori_pitch)

        time.sleep(0.2)

    # Ferma i motori coinvolti
    set_multiple_servos_pwm(master, [(5, 1500), (6, 1500), (7, 1500), (8, 1500)])
    print("[INFO] Motori pitch fermati.")


def rollMovement(master, target_roll_deg, timeout=15, tolerance=2.0, correction_pwm=100):
    """
    Porta il ROV al roll desiderato usando i motori laterali.
    - target_roll_deg: angolo in gradi (positivo = rotazione verso destra, negativo = verso sinistra)
    - timeout: tempo massimo per raggiungerlo
    - tolerance: margine di errore in gradi
    - correction_pwm: forza di correzione PWM
    """
    print(f"[INFO] Target roll: {target_roll_deg:.2f}°")

    start_time = time.time()

    while time.time() - start_time < timeout:
        # Leggi orientamento
        attitude_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if not attitude_msg:
            continue

        # Converti roll in gradi
        current_roll = math.degrees(attitude_msg.roll) - 90
        delta = target_roll_deg + current_roll

        print(f"[INFO] Roll attuale: {current_roll:.2f}° | Delta: {delta:.2f}°")

        if abs(delta) <= tolerance:
            print("[INFO] Roll raggiunto.")
            break

        # PWM correttivo
        # Nota: aggiusta i motori in base alla tua configurazione fisica!
        if delta > 0:
            # Serve inclinazione a destra → più spinta a sinistra
            pwm_left  = 1500 + correction_pwm
            pwm_right = 1500 - correction_pwm
        else:
            # Serve inclinazione a sinistra → più spinta a destra
            pwm_left  = 1500 - correction_pwm
            pwm_right = 1500 + correction_pwm

        # Esempio: motori 1 e 3 = lato sinistro, 2 e 4 = lato destro
        motori_roll = [
            (5, pwm_left),
            (6, pwm_left),
            (7, pwm_right),
            (8, pwm_right)
        ]
        set_multiple_servos_pwm(master, motori_roll)

        time.sleep(0.2)

    # Ferma i motori coinvolti
    set_multiple_servos_pwm(master, [(5, 1500), (6, 1500), (7, 1500), (8, 1500)])
    print("[INFO] Motori roll fermati.")



def forwardYawThrottleControl(master, 
                              duration, 
                              forward_power=1600, 
                              correction_pwm_yaw=10, 
                              correction_pwm_depth=50, 
                              yaw_tolerance=2, 
                              depth_tolerance=0.005):
    """
    Avanza in avanti mantenendo direzione (yaw) e profondità costanti.
    """
    print("[INFO] Avvio movimento controllato: avanti + yaw stabile + profondità stabile")

    # Leggi yaw iniziale
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not att_msg:
        print("[ERRORE] Nessun dato ATTITUDE ricevuto.")
        return
    initial_yaw = math.degrees(att_msg.yaw) % 360

    # Leggi profondità iniziale
    pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not pressure_msg:
        print("[ERRORE] Nessun dato SCALED_PRESSURE ricevuto.")
        return
    target_depth = (pressure_msg.press_abs - 1013.25) / 100.0

    print(f"[INFO] Yaw iniziale: {initial_yaw:.2f}°")
    print(f"[INFO] Profondità target: {target_depth:.2f} m")

    # Inizio movimento
    current_depth = (pressure_msg.press_abs - 1013.25) / 100.0
    start_time = time.time()
    while time.time() - start_time < duration:
        # Dati attuali
        att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.3)
        pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.3)
        if not att_msg or not pressure_msg:
            continue

        # --- YAW CONTROL ---
        current_yaw = math.degrees(att_msg.yaw) % 360
        delta_yaw = (initial_yaw - current_yaw + 180) % 360 - 180  # [-180, 180]

        yaw_pwm_offset = 0
        if delta_yaw > yaw_tolerance:
            yaw_pwm_offset = -correction_pwm_yaw  # correzione antioraria
        elif delta_yaw < -yaw_tolerance:
            yaw_pwm_offset = correction_pwm_yaw  # correzione oraria

        # --- DEPTH CONTROL ---
        delta_depth = target_depth - current_depth

        depth_pwm = 1500
        if delta_depth > depth_tolerance:
            depth_pwm += correction_pwm_depth  # scende
        elif delta_depth < depth_tolerance:
            depth_pwm -= correction_pwm_depth  # sale

        # --- MOTORI ORIZZONTALI ---
        forward_delta = forward_power - 1500
        motori_orizzontali = [
            (1, 1500 + forward_delta + yaw_pwm_offset),
            (2, 1500 + forward_delta - yaw_pwm_offset),
            (3, 1500 + forward_delta + yaw_pwm_offset),
            (4, 1500 + forward_delta - yaw_pwm_offset)
        ]

        # --- MOTORI VERTICALI ---
        motori_verticali = [(5, depth_pwm), (6, 3000 - depth_pwm), (7, 3000 - depth_pwm), (8, depth_pwm)]

        # Invio comandi
        set_multiple_servos_pwm(master, motori_orizzontali + motori_verticali)

        print(f"[INFO] Yaw: {current_yaw:.2f}° | ΔYaw: {delta_yaw:.2f}° | Profondità: {current_depth:.2f} m")

    # Ferma tutti i motori
    stop_all = [(i, 1500) for i in range(1, 9)]
    set_multiple_servos_pwm(master, stop_all)
    print("[INFO] Movimento completato. Tutti i motori fermati.")



def forwardStabilized(master, duration, forward_power=1580,
                                       correction_pwm_yaw=10,
                                       correction_pwm_depth=50,
                                       correction_pwm_pitch=25,
                                       correction_pwm_roll=25,
                                       yaw_tolerance=5,
                                       depth_tolerance=0.005,
                                       pitch_tolerance=3.0,
                                       roll_tolerance=3.0):
    """
    Movimento in avanti controllato mantenendo:
    - rotta (yaw)
    - profondità
    - pitch (inclinazione verticale)
    - roll (inclinazione orizzontale laterale)
    """

    print("[INFO] Avvio movimento controllato completo (forward + stabilizzazione completa)")

    # Lettura iniziale
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not att_msg:
        print("[ERRORE] Nessun dato ATTITUDE ricevuto.")
        return

    pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not pressure_msg:
        print("[ERRORE] Nessun dato di pressione ricevuto.")
        return

    initial_yaw = math.degrees(att_msg.yaw) % 360
    target_pitch = math.degrees(att_msg.pitch)
    target_roll = math.degrees(att_msg.roll)
    target_depth = (pressure_msg.press_abs - 1013.25) / 100.0

    print(f"[INFO] Yaw iniziale: {initial_yaw:.2f}° | Pitch: {target_pitch:.2f}° | Roll: {target_roll:.2f}° | Profondità: {target_depth:.2f} m")

    start_time = time.time()
    while time.time() - start_time < duration:
        att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.3)
        pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.3)
        if not att_msg or not pressure_msg:
            continue

        # --- YAW ---
        current_yaw = math.degrees(att_msg.yaw) % 360
        delta_yaw = (initial_yaw - current_yaw + 180) % 360 - 180
        yaw_pwm_offset = 0
        if delta_yaw > yaw_tolerance:
            yaw_pwm_offset = -correction_pwm_yaw
        elif delta_yaw < -yaw_tolerance:
            yaw_pwm_offset = correction_pwm_yaw

        # --- DEPTH ---
        current_depth = (pressure_msg.press_abs - 1013.25) / 100.0
        delta_depth = target_depth - current_depth
        depth_pwm = 1500
        if delta_depth > depth_tolerance:
            depth_pwm += correction_pwm_depth
        elif delta_depth < -depth_tolerance:
            depth_pwm -= correction_pwm_depth

        # --- PITCH ---
        current_pitch = math.degrees(att_msg.pitch)
        delta_pitch = target_pitch - current_pitch
        pwm_front = 1500
        pwm_back = 1500
        if abs(delta_pitch) > pitch_tolerance:
            if delta_pitch > 0:  # Nose up
                pwm_front -= correction_pwm_pitch
                pwm_back  += correction_pwm_pitch
            else:  # Nose down
                pwm_front += correction_pwm_pitch
                pwm_back  -= correction_pwm_pitch


        # --- ROLL ---
        current_roll = math.degrees(att_msg.roll)
        delta_roll = target_roll - current_roll
        pwm_left = 1500
        pwm_right = 1500
        if abs(delta_roll) > roll_tolerance:
            if delta_roll > 0:  # incl. left, spingi destra
                pwm_left  += correction_pwm_roll
                pwm_right -= correction_pwm_roll
            else:  # incl. right
                pwm_left  -= correction_pwm_roll
                pwm_right += correction_pwm_roll

        # --- MOTORI ORIZZONTALI (1-4): AVANTI + YAW ---
        forward_delta = forward_power - 1500
        motori_orizzontali = [
            (1, 1500 + forward_delta + yaw_pwm_offset),
            (2, 1500 + forward_delta - yaw_pwm_offset),
            (3, 1500 - forward_delta - yaw_pwm_offset),
            (4, 1500 - forward_delta + yaw_pwm_offset)
        ]



        # --- MOTORI VERTICALI (5-8): THROTTLE + PITCH + ROLL ---
        # Assegnazione (ipotetica):
        # - 5: anteriore sinistra
        # - 6: anteriore destra
        # - 7: posteriore sinistra
        # - 8: posteriore destra
        motori_verticali = [
            (5, depth_pwm + pwm_back + pwm_left  - 1500),  # 1500 + pitch + roll
            (6, depth_pwm + pwm_front + pwm_left - 1500),
            (7, depth_pwm + pwm_back + pwm_right  - 1500),
            (8, depth_pwm + pwm_front + pwm_right - 1500),
        ]


        # Invia comandi
        set_multiple_servos_pwm(master, motori_orizzontali + motori_verticali)

        print(f"[INFO] Yaw: {current_yaw:.2f}° | ΔYaw: {delta_yaw:.2f}° | Depth: {current_depth:.2f} m | ΔDepth: {delta_depth:.2f} m")
        print(f"[INFO] Pitch: {current_pitch:.2f}° | ΔPitch: {delta_pitch:.2f}° | Roll: {current_roll:.2f}° | ΔRoll: {delta_roll:.2f}°")
        time.sleep(0.2)

    # Ferma tutti i motori
    stop_all = [(i, 1500) for i in range(1, 9)]
    set_multiple_servos_pwm(master, stop_all)
    print("[INFO] Movimento completato. Tutti i motori fermati.")



def yawDepthControl(master, degrees, power=1600, yaw_tolerance=5, timeout_sec=10,
                    depth_tolerance=0.1, correction_pwm_depth=40):
    """
    Ruota il ROV di un certo numero di gradi (yaw) mantenendo la profondità.
    - degrees: positivo = orario, negativo = antiorario
    - power: intensità della rotazione (PWM)
    - depth_tolerance: soglia massima di variazione consentita per la profondità (in metri)
    """
    print(f"[INFO] Rotazione richiesta: {degrees:.1f}° con mantenimento profondità")

    # Riceve dati iniziali
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not att_msg or not press_msg:
        print("[ERRORE] Dati ATTITUDE o pressione non disponibili.")
        return

    initial_yaw = math.degrees(att_msg.yaw) % 360
    target_yaw = (initial_yaw + degrees) % 360
    target_depth = (press_msg.press_abs - 1013.25) / 100.0

    print(f"[INFO] Yaw iniziale: {initial_yaw:.2f}°, target: {target_yaw:.2f}°")
    print(f"[INFO] Profondità iniziale: {target_depth:.2f} m")

    start_time = time.time()
    direction = 1 if degrees > 0 else -1
    pwm_delta = abs(power - 1500)

    while time.time() - start_time < timeout_sec:
        att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.3)
        press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.3)
        if not att_msg or not press_msg:
            continue

        # YAW CONTROL
        current_yaw = math.degrees(att_msg.yaw) % 360
        delta_yaw = (target_yaw - current_yaw + 180) % 360 - 180

        if abs(delta_yaw) <= yaw_tolerance:
            break

        motor_1 = 1500 + direction * pwm_delta
        motor_2 = 1500 - direction * pwm_delta
        motor_3 = 1500 - direction * pwm_delta
        motor_4 = 1500 + direction * pwm_delta

        motori_yaw = [
            (1, motor_1),
            (2, motor_2),
            (3, motor_3),
            (4, motor_4)
        ]

        # DEPTH CONTROL
        current_depth = (press_msg.press_abs - 1013.25) / 100.0
        delta_depth = target_depth - current_depth
        pwm_vertical = 1500
        if delta_depth > depth_tolerance:
            pwm_vertical += correction_pwm_depth
        elif delta_depth < -depth_tolerance:
            pwm_vertical -= correction_pwm_depth

        motori_verticali = [
            (5, pwm_vertical),
            (6, pwm_vertical),
            (7, pwm_vertical),
            (8, pwm_vertical)
        ]

        # Invia comandi combinati
        set_multiple_servos_pwm(master, motori_yaw + motori_verticali)

        print(f"[INFO] Yaw attuale: {current_yaw:.2f}°, ΔYaw: {delta_yaw:.2f}° | Profondità: {current_depth:.2f} m, ΔDepth: {delta_depth:.2f} m")
        time.sleep(0.1)

    # Ferma tutti i motori
    stop = [(i, 1500) for i in range(1, 9)]
    set_multiple_servos_pwm(master, stop)
    print("[INFO] Rotazione completata e motori fermati.")



def yawStabilized(master,
                  degrees,
                  power=1600,
                  yaw_tolerance=3,
                  timeout_sec=15,
                  pitch_tolerance=2.0,
                  roll_tolerance=2.0,
                  depth_tolerance=0.1,
                  correction_pwm_pitch=30,
                  correction_pwm_roll=30,
                  correction_pwm_throttle=40):
    """
    Ruota il ROV di un certo numero di gradi (yaw) mantenendo assetto e profondità.
    Controlla simultaneamente: yaw, pitch, roll, throttle.

    - degrees: gradi di rotazione (positivi orario, negativi antiorario)
    - power: forza di rotazione (PWM)
    """
    print(f"[INFO] Inizio rotazione stabilizzata: {degrees:.1f}°")

    # Leggi assetto e profondità iniziali
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not att_msg or not press_msg:
        print("[ERRORE] Impossibile leggere assetto o pressione iniziale.")
        return

    initial_yaw = math.degrees(att_msg.yaw) % 360
    target_yaw = (initial_yaw + degrees) % 360
    target_pitch = math.degrees(att_msg.pitch)
    target_roll = math.degrees(att_msg.roll)
    target_depth = (press_msg.press_abs - 1013.25) / 100.0

    print(f"[INFO] Target Yaw: {target_yaw:.2f}°, Pitch: {target_pitch:.2f}°, Roll: {target_roll:.2f}°, Profondità: {target_depth:.2f} m")

    start_time = time.time()
    direction = 1 if degrees > 0 else -1
    pwm_yaw_delta = abs(power - 1500)

    while time.time() - start_time < timeout_sec:
        att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
        press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.2)
        if not att_msg or not press_msg:
            continue

        # --- YAW CONTROL ---
        current_yaw = math.degrees(att_msg.yaw) % 360
        delta_yaw = (target_yaw - current_yaw + 180) % 360 - 180

        # Stop condition
        if abs(delta_yaw) <= yaw_tolerance:
            break

        motor_1 = 1500 + direction * pwm_yaw_delta
        motor_2 = 1500 - direction * pwm_yaw_delta
        motor_3 = 1500 - direction * pwm_yaw_delta
        motor_4 = 1500 + direction * pwm_yaw_delta
        motori_yaw = [(1, motor_1), (2, motor_2), (3, motor_3), (4, motor_4)]

        # --- DEPTH CONTROL ---
        current_depth = (press_msg.press_abs - 1013.25) / 100.0
        delta_depth = target_depth - current_depth
        pwm_vertical = 1500
        if delta_depth > depth_tolerance:
            pwm_vertical += correction_pwm_throttle
        elif delta_depth < -depth_tolerance:
            pwm_vertical -= correction_pwm_throttle
        motori_verticali = [(5, pwm_vertical), (6, pwm_vertical), (7, pwm_vertical), (8, pwm_vertical)]

        # --- PITCH CONTROL ---
        current_pitch = math.degrees(att_msg.pitch)
        delta_pitch = target_pitch - current_pitch
        if delta_pitch > pitch_tolerance:
            pwm_front = 1500 - correction_pwm_pitch
            pwm_back = 1500 + correction_pwm_pitch
        elif delta_pitch < -pitch_tolerance:
            pwm_front = 1500 + correction_pwm_pitch
            pwm_back = 1500 - correction_pwm_pitch
        else:
            pwm_front = pwm_back = 1500
        motori_pitch = [(5, pwm_front), (6, pwm_front), (7, pwm_back), (8, pwm_back)]

        # --- ROLL CONTROL ---
        current_roll = math.degrees(att_msg.roll)
        delta_roll = target_roll - current_roll
        if delta_roll > roll_tolerance:
            pwm_left = 1500 - correction_pwm_roll
            pwm_right = 1500 + correction_pwm_roll
        elif delta_roll < -roll_tolerance:
            pwm_left = 1500 + correction_pwm_roll
            pwm_right = 1500 - correction_pwm_roll
        else:
            pwm_left = pwm_right = 1500
        motori_roll = [(5, pwm_left), (6, pwm_right), (7, pwm_left), (8, pwm_right)]

        # --- COMBINAZIONE COMANDI ---
        # Combina gli effetti su motori 5–8 da pitch, roll e depth → media dei valori PWM
        combined_vertical = []
        for i in range(5, 9):
            pwm_values = []
            for m_list in [motori_verticali, motori_pitch, motori_roll]:
                for (ch, pwm) in m_list:
                    if ch == i:
                        pwm_values.append(pwm)
            avg_pwm = sum(pwm_values) / len(pwm_values)
            combined_vertical.append((i, int(avg_pwm)))

        # Invio finale
        set_multiple_servos_pwm(master, motori_yaw + combined_vertical)

        print(f"[INFO] Yaw: {current_yaw:.2f}°, ΔYaw: {delta_yaw:.2f}° | Depth: {current_depth:.2f} m, Pitch: {current_pitch:.2f}°, Roll: {current_roll:.2f}°")
        time.sleep(0.1)

    # Ferma tutti i motori
    set_multiple_servos_pwm(master, [(i, 1500) for i in range(1, 9)])
    print("[INFO] Rotazione completata. Motori stabilizzati.")




def rollStabilized(master, target_roll_deg, timeout=15, tolerance=2.0, roll_pwm=30, depth_tolerance=0.1, att_tolerance=2.0):
    """
    Esegue una rotazione sul roll mantenendo profondità, pitch e yaw costanti.
    """
    print(f"[INFO] Target roll: {target_roll_deg:.2f}°")
    start_time = time.time()

    # Ottieni profondità iniziale
    depth_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not depth_msg:
        print("[ERRORE] Nessun dato di profondità ricevuto.")
        return

    initial_pressure = depth_msg.press_abs
    target_depth = (initial_pressure - 1013.25) / 100.0

    # Ottieni orientamento iniziale
    attitude_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not attitude_msg:
        print("[ERRORE] Nessun dato ATTITUDE ricevuto.")
        return

    target_pitch = math.degrees(attitude_msg.pitch)
    target_yaw = math.degrees(attitude_msg.yaw) % 360

    while time.time() - start_time < timeout:
        # Leggi nuovi dati
        att = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        depth = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.5)

        if not att or not depth:
            continue

        # ATTITUDE attuale
        current_roll  = math.degrees(att.roll)
        current_pitch = math.degrees(att.pitch)
        current_yaw   = math.degrees(att.yaw) % 360
        current_depth = (depth.press_abs - 1013.25) / 100.0

        delta_roll  = target_roll_deg - current_roll
        delta_pitch = target_pitch - current_pitch
        delta_yaw   = (target_yaw - current_yaw + 180) % 360 - 180
        delta_depth = target_depth - current_depth

        print(f"[INFO] Roll: {current_roll:.2f} Δ{delta_roll:.1f} | Pitch Δ{delta_pitch:.1f} | Yaw Δ{delta_yaw:.1f} | Depth Δ{delta_depth:.2f}")

        # === ROLL ===
        if abs(delta_roll) > tolerance:
            if delta_roll > 0:
                pwm_left  = 1500 + roll_pwm
                pwm_right = 1500 - roll_pwm
            else:
                pwm_left  = 1500 - roll_pwm
                pwm_right = 1500 + roll_pwm
        else:
            pwm_left = pwm_right = 1500

        # === DEPTH CONTROL (con motori che attualmente contribuiscono alla Z) ===
        depth_pwm = 1500
        correction_pwm_depth = 40
        if abs(delta_depth) > depth_tolerance:
            if delta_depth > 0:
                depth_pwm += correction_pwm_depth  # scende
            else:
                depth_pwm -= correction_pwm_depth  # sale

        # Calcolo pesi in base al roll
        roll_rad = math.radians(current_roll)
        factor_5_8 = abs(math.cos(roll_rad))         # motori verticali "puri"
        factor_1_4 = 1.0 - factor_5_8                # motori "forward" ora usati per z

        # Calcolo PWM per ciascun gruppo
        pwm_5_8 = int(1500 + (depth_pwm - 1500) * factor_5_8)
        pwm_1_4 = int(1500 + (depth_pwm - 1500) * factor_1_4)

        vertical_motors = [
            (1, pwm_1_4), (2, pwm_1_4), (3, pwm_1_4), (4, pwm_1_4),
            (5, pwm_5_8), (6, pwm_5_8), (7, pwm_5_8), (8, pwm_5_8)
        ]


        # === PITCH & YAW STABILIZATION (neutro se sotto soglia) ===
        pitch_pwm = yaw_pwm = 1500
        pitch_correction = 20
        yaw_correction = 20

        if abs(delta_pitch) > att_tolerance:
            pitch_pwm += pitch_correction if delta_pitch > 0 else -pitch_correction

        if abs(delta_yaw) > att_tolerance:
            yaw_pwm += yaw_correction if delta_yaw > 0 else -yaw_correction

        # Qui potresti decidere quali motori usare per correggere pitch e yaw
        # Esempio: solo una leggera modifica a motori 1-4
        motors_stabilize = [
            (1, yaw_pwm),  # applichi yaw
            (2, yaw_pwm),
            (3, pitch_pwm),  # applichi pitch
            (4, pitch_pwm)
        ]

        # === Motori per eseguire il roll (5–8) ===
        motors_roll = [
            (5, pwm_left),
            (6, pwm_left),
            (7, pwm_right),
            (8, pwm_right)
        ]

        # Combina e invia tutti
        set_multiple_servos_pwm(master, motors_roll + motors_stabilize + vertical_motors)

        time.sleep(0.2)

        if abs(delta_roll) <= tolerance:
            print("[INFO] Roll target raggiunto con stabilizzazione.")
            break

    # Ferma tutti i motori
    neutral = [(i, 1500) for i in range(1, 9)]
    set_multiple_servos_pwm(master, neutral)
    print("[INFO] Tutti i motori fermati.")




def pitchStabilized(master, target_pitch_deg, timeout=15, tolerance_pitch=2.0, tolerance_depth=0.1, correction_pwm=40):
    """
    Esegue un movimento di pitch mantenendo la profondità attuale del ROV.
    - target_pitch_deg: angolo di pitch da raggiungere (in gradi)
    """
    print(f"[INFO] Target pitch: {target_pitch_deg:.2f}°")

    # --- Leggi profondità attuale per usarla come target ---
    pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not pressure_msg:
        print("[ERRORE] Nessun dato di pressione disponibile.")
        return

    current_pressure = pressure_msg.press_abs  # in mbar
    target_depth_m = (current_pressure - 1013.25) / 100.0  # ≈ metri

    print(f"[INFO] Profondità attuale rilevata: {target_depth_m:.2f} m (impostata come target)")

    start_time = time.time()

    while time.time() - start_time < timeout:
        # --- Attitude ---
        attitude_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.5)
        if not attitude_msg:
            continue
        current_pitch = math.degrees(attitude_msg.pitch)
        delta_pitch = target_pitch_deg - current_pitch

        # --- Profondità ---
        pressure_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.5)
        if not pressure_msg:
            continue
        current_depth = (pressure_msg.press_abs - 1013.25) / 100.0
        delta_depth = target_depth_m - current_depth

        print(f"[INFO] Pitch: {current_pitch:.2f}° | ΔPitch: {delta_pitch:.2f}° | Depth: {current_depth:.2f} m | ΔDepth: {delta_depth:.2f} m")

        if abs(delta_pitch) <= tolerance_pitch and abs(delta_depth) <= tolerance_depth:
            print("[INFO] Pitch e profondità raggiunti.")
            break

        # --- Correzione Pitch ---
        if delta_pitch > 0:
            pwm_front = 1500 - correction_pwm
            pwm_back  = 1500 + correction_pwm
        else:
            pwm_front = 1500 + correction_pwm
            pwm_back  = 1500 - correction_pwm

        motori_pitch = [
            (5, pwm_front), (6, pwm_front),
            (7, pwm_back), (8, pwm_back)
        ]

        # --- Correzione profondità dinamica ---
        pwm_depth = 1500
        if delta_depth > tolerance_depth:
            pwm_depth += correction_pwm
        elif delta_depth < -tolerance_depth:
            pwm_depth -= correction_pwm

        pitch_rad = math.radians(current_pitch)
        weight_vertical = abs(math.cos(pitch_rad))
        weight_forward = abs(math.sin(pitch_rad))

        pwm_vertical = int(pwm_depth * weight_vertical + 1500 * (1 - weight_vertical))
        pwm_forward = int(pwm_depth * weight_forward + 1500 * (1 - weight_forward))

        motori_verticali = [(5, pwm_vertical), (6, pwm_vertical), (7, pwm_vertical), (8, pwm_vertical)]
        motori_forward = [(1, pwm_forward), (2, pwm_forward), (3, pwm_forward), (4, pwm_forward)]

        # --- Invia comandi ---
        set_multiple_servos_pwm(master, motori_pitch + motori_forward)
        time.sleep(0.2)

    # Ferma tutti i motori
    set_multiple_servos_pwm(master, [(i, 1500) for i in range(1, 9)])
    print("[INFO] Movimento pitch completato.")



def throttleStabilized(master,
                        target_depth,
                        power=1600,
                        timeout_sec=20,
                        depth_tolerance=0.005,
                        pitch_tolerance=2.0,
                        roll_tolerance=2.0,
                        yaw_tolerance=3.0,
                        correction_pwm_pitch=25,
                        correction_pwm_roll=25,
                        correction_pwm_yaw=10):
    """
    Muove il ROV in verticale (throttle) fino a target_depth mantenendo stabili yaw, pitch e roll.

    - target_depth: profondità target in metri
    - power: forza verticale (PWM base per salita/discesa)
    """
    print(f"[INFO] Movimento verticale stabilizzato verso {target_depth:.4f} m")

    # Leggi stato iniziale
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=2)
    if not att_msg or not press_msg:
        print("[ERRORE] Impossibile leggere assetto o pressione iniziale.")
        return

    # Valori target per stabilizzazione
    target_yaw = math.degrees(att_msg.yaw) % 360
    target_pitch = math.degrees(att_msg.pitch)
    target_roll = math.degrees(att_msg.roll)

    print(f"[INFO] Target Yaw: {target_yaw:.2f}°, Pitch: {target_pitch:.2f}°, Roll: {target_roll:.2f}°")


    current_depth = (press_msg.press_abs - 1013.25) / 100.0
    delta_depth = target_depth - current_depth
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
        press_msg = master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=0.2)
        if not att_msg or not press_msg:
            continue

        # Se profondità raggiunta
        if abs(delta_depth) <= depth_tolerance:
            print("[INFO] Profondità target raggiunta.")
            break

        pwm_vertical = power if delta_depth > 0 else (3000 - power)  # >0 scende, <0 sale

        motori_verticali = [(5, pwm_vertical), (6, pwm_vertical), (7, pwm_vertical), (8, pwm_vertical)]

        # --- PITCH CONTROL ---
        current_pitch = math.degrees(att_msg.pitch)
        delta_pitch = target_pitch - current_pitch
        if delta_pitch > pitch_tolerance:
            pwm_front = 1500 - correction_pwm_pitch
            pwm_back = 1500 + correction_pwm_pitch
        elif delta_pitch < -pitch_tolerance:
            pwm_front = 1500 + correction_pwm_pitch
            pwm_back = 1500 - correction_pwm_pitch
        else:
            pwm_front = pwm_back = 1500
        motori_pitch = [(5, pwm_front), (6, pwm_front), (7, pwm_back), (8, pwm_back)]

        # --- ROLL CONTROL ---
        current_roll = math.degrees(att_msg.roll)
        delta_roll = target_roll - current_roll
        if delta_roll > roll_tolerance:
            pwm_left = 1500 - correction_pwm_roll
            pwm_right = 1500 + correction_pwm_roll
        elif delta_roll < -roll_tolerance:
            pwm_left = 1500 + correction_pwm_roll
            pwm_right = 1500 - correction_pwm_roll
        else:
            pwm_left = pwm_right = 1500
        motori_roll = [(5, pwm_left), (6, pwm_right), (7, pwm_left), (8, pwm_right)]

        # --- YAW CONTROL (solo stabilizzazione, non rotazione) ---
        current_yaw = math.degrees(att_msg.yaw) % 360
        delta_yaw = (target_yaw - current_yaw + 180) % 360 - 180
        if delta_yaw > yaw_tolerance:
            pwm_yaw = 1500 - correction_pwm_yaw
        elif delta_yaw < -yaw_tolerance:
            pwm_yaw = 1500 + correction_pwm_yaw
        else:
            pwm_yaw = 1500
        motori_yaw = [(1, pwm_yaw), (2, pwm_yaw), (3, pwm_yaw), (4, pwm_yaw)]

        # --- COMBINAZIONE COMANDI ---
        combined_vertical = []
        for i in range(5, 9):
            pwm_values = []
            for m_list in [motori_verticali, motori_pitch, motori_roll]:
                for (ch, pwm) in m_list:
                    if ch == i:
                        pwm_values.append(pwm)
            avg_pwm = sum(pwm_values) / len(pwm_values)
            combined_vertical.append((i, int(avg_pwm)))

        # Invio ai motori
        set_multiple_servos_pwm(master, motori_yaw + combined_vertical)

        print(f"[INFO] Depth: {current_depth:.2f} m (Δ {delta_depth:.2f}), "
              f"Pitch: {current_pitch:.2f}°, Roll: {current_roll:.2f}°, Yaw: {current_yaw:.2f}°")

        time.sleep(0.1)

    # Ferma motori
    set_multiple_servos_pwm(master, [(i, 1500) for i in range(1, 9)])
    print("[INFO] Movimento verticale completato e stabilizzato.")




def square_path(master, side_duration=5):
    """
    Esegue un percorso a quadrato utilizzando controllo di yaw e profondità.
    """
    print("Inizio percorso quadrato...\n")
    for i in range(4):
        print(f"Lato {i+1}: avanti")
        forwardStabilized(master, duration=side_duration, forward_power=1600)

        print(f"Rotazione di 90° (destra)")
        yawStabilized(master, degrees=90, power=1600)

    print("Percorso completato.")


def triangle_path(master, side_duration=5):
    """
    Esegue un percorso a triangolo utilizzando controllo di yaw e profondità.
    """
    print("Inizio percorso triangolare...\n")
    for i in range(3):
        print(f"Lato {i+1}: avanti")
        forwardStabilized(master, duration=side_duration, forward_power=1600)

        print(f"Rotazione di 60° (destra)")
        yawStabilized(master, degrees=60, power=1600)

    print("Percorso completato.")


def circle_path(master, side_duration=0.5):
    """
    Esegue un percorso circolare utilizzando controllo di yaw e profondità.
    """
    print("Inizio percorso circolare...\n")
    for i in range(15):
        forwardStabilized(master, duration=side_duration, forward_power=1600)

        print(f"Rotazione di 90° (destra)")
        yawStabilized(master, degrees=24, power=1600)

    print("Percorso completato.")


def complex_path(master, straight_duration_sec=5, target_depth=0.5, lapsNumber = 2):
    print("Inizio percorso complesso...\n")
    for i in range(lapsNumber):
        print(f"Avanzamento")
        forwardStabilized(master, duration=straight_duration_sec, forward_power=1600)
        print(f"Discesa")
        throttleStabilized(master, target_depth=target_depth, power=1600)
        print(f"Rotazione di 180°")
        yawStabilized(master, degrees=180, power=1600)
        print(f"Ritorno")
        forwardStabilized(master, duration=straight_duration_sec, forward_power=1600)
        print(f"Ascesa")
        throttleStabilized(master, target_depth=0.1)
        print(f"Rotazione di 180°")
        yawStabilized(master, degrees=180, power=1600)
        print(f"Lap {i} completato")

    print("Percorso completato.")





def main():
    # 1. Connessione MAVLink
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()
    print("Connessione stabilita con il veicolo.")

    # Imposta tutti i motori a Disabled
    set_param(master)

    try:
        arm(master)
        #forwardMovement(master, duration=3, power=1900)
        #yawMovement(master, degrees=-90, power= 1600)  # ruota 90° a destra
        #throttleMovement(master, 0.5)
        #pitchMovement(master, target_pitch_deg=90)
        #rollMovement(master, target_roll_deg=45)

        #forwardYawThrottleControl(master, 20, 1600)
        #forwardStabilized(master, 8, 1600)

        throttleStabilized(master, 0.16, 1600)

        #yawDepthControl(master, 90, 1600)
        #yawStabilized(master, 90, 1600)

        #rollStabilized(master, 90)

        #pitchStabilized(master, 90)

        #circle_path(master)
        #triangle_path(master)
        #circle_path(master)
        #complex_path(master)


    except KeyboardInterrupt:
        print("\nInterruzione manuale (Ctrl+C) ricevuta.")

    finally:
        # Ferma tutti i motori
        motori_neutri = [(i, 1500) for i in range(1, 9)]
        set_multiple_servos_pwm(master, motori_neutri)

        # Disarma il veicolo
        disarm(master)
        print("Motori fermati e veicolo disarmato.")

    


if __name__ == "__main__":
    main()