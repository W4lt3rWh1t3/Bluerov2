import utilities.rc_override as rc
import time
import csv
import math
import threading


# Variabile globale del ROV
sub = None


def sub_forward(drone):
    time.sleep(0.5)
    drone.send_rc(forward=1590)
    drone.status_loop(30)
    drone.clear_motion()


def sub_left(drone):
    time.sleep(1)
    drone.send_rc(yaw= 1455)
    drone.status_loop(3)
    drone.clear_motion()


def sub_right(drone):
    time.sleep(1)
    drone.send_rc(yaw= 1545)
    drone.status_loop(3)
    drone.clear_motion()
    

def forward_with_yaw_and_depth_hold(duration_sec=5, motor_power=1500):
    """
    Esegue un movimento in avanti per un tempo specificato, mantenendo costanti 
    l'orientamento (yaw) e la profondità iniziale.

    Il sistema:
    1. Legge lo yaw e la profondità iniziale all'inizio del movimento.
    2. Durante l'avanzamento, controlla continuamente lo yaw e la profondità.
    3. Applica piccole correzioni via RC (yaw e throttle) per restare nei limiti di tolleranza.
    4. Interrompe il movimento dopo il tempo specificato o in caso di errore di lettura sensori.

    Parametri:
    - duration_sec (int): Durata del movimento in avanti, in secondi.
    - motor_power (int): Potenza RC per il movimento in avanti (tipicamente >1500 per avanzare).
    """
    print('Avvio forward controllato\n')

    yaw_tolerance_deg = 2
    depth_tolerance_m = 0.15
    rc_forward = motor_power
    rc_yaw_neutral = 1500
    rc_throttle_neutral = 1500
    yaw_correction = 5
    depth_correction = 40

    # Lettura yaw iniziale
    attitude_msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not attitude_msg:
        print("Errore: impossibile leggere lo yaw iniziale.")
        return
    initial_yaw = math.degrees(attitude_msg.yaw)

    # Lettura profondità iniziale
    msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=2)
    if msg:
        initial_depth = depth_conversion(msg.press_abs)  # CONVERSIONE IN METRI
    else:
        print("Errore: impossibile leggere la profondità iniziale.")
        return
    print(f"Avvio avanzamento per {duration_sec}s verso profondità {initial_depth:.3f} m")

    start_time = time.time()
    while time.time() - start_time < duration_sec:
        rc_yaw = rc_yaw_neutral
        rc_throttle = rc_throttle_neutral

        # Lettura stato attuale
        attitude_msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
        depth_msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=2)

        if depth_msg:
            current_depth = depth_conversion(depth_msg.press_abs)  # IN METRI
            delta_depth = current_depth - initial_depth
            print(f"Profondità attuale: {current_depth:.3f} m")

            if delta_depth > depth_tolerance_m:
                rc_throttle = rc_throttle_neutral + depth_correction
            elif delta_depth < -depth_tolerance_m:
                rc_throttle = rc_throttle_neutral - depth_correction

        if attitude_msg:
            current_yaw = math.degrees(attitude_msg.yaw)
            delta_yaw = (current_yaw - initial_yaw + 180) % 360 - 180
            print(f"Orientamento attuale: {current_yaw:.3f}°")

            if delta_yaw > yaw_tolerance_deg:
                rc_yaw = rc_yaw_neutral - yaw_correction
            elif delta_yaw < -yaw_tolerance_deg:
                rc_yaw = rc_yaw_neutral + yaw_correction

        # Invia comandi RC
        sub.send_rc(forward=rc_forward, yaw=rc_yaw, throttle=rc_throttle)
        time.sleep(0.05)

    sub.clear_motion()
    print("Movimento completato.")


def depth_conversion(mbar_pressure, atm_pressure=1018, water_density=1025):
    """
    Converte una pressione assoluta (in mbar) in una stima di profondità (in metri),
    assumendo che la pressione provenga da un sensore immerso in acqua.

    La conversione tiene conto della pressione atmosferica di riferimento e della densità del fluido.
    È utile per calcolare la profondità sommersa di un drone subacqueo partendo dai dati del sensore di pressione.

    Parametri:
    - mbar_pressure (float): Pressione assoluta letta dal sensore (in mbar).
    - atm_pressure (float): Pressione atmosferica di riferimento al livello dell’acqua (default: 1018 mbar).
    - water_density (float): Densità del fluido (in kg/m³), default 1025 per acqua salata.
    """
    g = 9.80665  # accelerazione di gravità in m/s^2
    pressione_diff = (mbar_pressure - atm_pressure) * 100  # da mbar a Pascal
    profondita = pressione_diff / (water_density * g)
    return max(profondita, 0)  # evita valori negativi se sei sopra il livello del mare


def depth_movement(target_depth_m, timeout_sec=15, depth_tolerance=0.15):
    """
    Esegue un movimento verticale verso una profondità target mantenendo lo yaw iniziale costante.

    Il sistema:
    1. Legge la profondità corrente da un sensore di pressione.
    2. Calcola la differenza rispetto alla profondità target.
    3. Regola la spinta verticale (throttle RC) in modo proporzionale per salire o scendere.
    4. Controlla costantemente lo yaw e lo corregge se necessario, entro una soglia di tolleranza.
    5. Termina il movimento quando la profondità è entro la tolleranza specificata o scade il timeout.

    Parametri:
    - target_depth_m (float): Profondità da raggiungere in metri.
    - timeout_sec (int): Tempo massimo concesso per raggiungere la profondità.
    - depth_tolerance (float): Tolleranza accettabile rispetto alla profondità target (in metri).
    """
    print(f"[DEPTH] Inizio movimento verso {target_depth_m:.3f} m mantenendo lo yaw.")

    yaw_tolerance_deg = 2
    rc_yaw_neutral = 1500
    rc_throttle = 1500
    rc_yaw = rc_yaw_neutral
    throttle_step = 100
    yaw_correction = 30

    # Ottieni yaw iniziale
    attitude_msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not attitude_msg:
        print("[DEPTH] Errore: impossibile leggere lo yaw iniziale.")
        return
    initial_yaw = math.degrees(attitude_msg.yaw)

    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        attitude_msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
        depth_msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=0.2)

        if not depth_msg:
            continue

        # Calcola profondità in metri
        current_depth = depth_conversion(depth_msg.press_abs)

        # Calcolo correzione profondità
        delta_depth = target_depth_m - current_depth
        if abs(delta_depth) <= depth_tolerance:
            print(f"[DEPTH] Profondità target raggiunta: {current_depth:.3f} m")
            break
        elif delta_depth > 0:
            rc_throttle = 1500 - throttle_step
        else:
            rc_throttle = 1500 + throttle_step

        # Correzione yaw per mantenere orientamento iniziale
        if attitude_msg:
            current_yaw = math.degrees(attitude_msg.yaw) 
            delta_yaw = (current_yaw - initial_yaw + 180) % 360 - 180
            rc_yaw = rc_yaw_neutral
            if delta_yaw > yaw_tolerance_deg:
                rc_yaw = rc_yaw_neutral - yaw_correction
            elif delta_yaw < -yaw_tolerance_deg:
                rc_yaw = rc_yaw_neutral + yaw_correction

        print(f"[DEPTH] Depth: {current_depth:.3f} m, Yaw: {current_yaw:.2f}°")

        sub.send_rc(throttle=rc_throttle, yaw=rc_yaw)
        time.sleep(0.05)

    sub.clear_motion()
    print("[DEPTH] Movimento di profondità completato.")


def map_angle_to_rc_speed(delta_yaw, rc_neutral=1500, max_speed=130, min_speed=80):
    """
    Converte un errore angolare (delta_yaw) in un segnale RC proporzionale da inviare al controllo di yaw.

    Il valore RC risultante rappresenta la velocità angolare richiesta per correggere l'orientamento del veicolo.
    La velocità cresce con l'errore angolare, ma è limitata da un minimo e un massimo per garantire controllo
    stabile e sicuro.

    Parametri:
    - delta_yaw (float): Differenza angolare tra yaw corrente e target (in gradi). Il segno indica la direzione.
    - rc_neutral (int): Valore RC neutro (tipicamente 1500), cioè nessun movimento.
    - max_speed (int): Offset massimo da rc_neutral (tipicamente 130).
    - min_speed (int): Offset minimo per garantire che il comando superi la soglia di movimento (tipicamente 80).
    """
    direction = 1 if delta_yaw > 0 else -1
    delta = abs(delta_yaw)
    
    # Proporzione (scaling)
    rc_offset = min(max_speed, max(min_speed, int((delta / 90.0) * max_speed)))
    
    return rc_neutral + direction * rc_offset


def rotate_by(degrees, yaw_tolerance_deg=10, timeout_sec=15):
    """
    Esegue una rotazione del veicolo (ad esempio un drone o ROV) lungo l'asse di yaw (imbardata)
    utilizzando un controllo proporzionale, fino a raggiungere l'angolo desiderato.

    Il sistema:
    1. Legge lo yaw corrente come angolo iniziale.
    2. Calcola lo yaw target sommando il valore di rotazione (in gradi).
    3. Regola dinamicamente la velocità angolare (yaw) tramite un controllo proporzionale
       in base alla distanza angolare residua.
    4. Si arresta quando la differenza angolare è entro una soglia (`yaw_tolerance_deg`) o 
       quando scade il timeout.

    Parametri:
    - degrees (float): Rotazione desiderata in gradi. Può essere positiva (orario) o negativa (antiorario).
    - yaw_tolerance_deg (float): Tolleranza in gradi per considerare completata la rotazione.
    - timeout_sec (int): Tempo massimo in secondi per completare la rotazione prima dell'interruzione.
    """
    print(f"[ROTATE] Inizio rotazione proporzionale di {degrees:.1f}°")

    msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if not msg:
        print("[ROTATE] Errore: impossibile leggere lo yaw iniziale.")
        return

    initial_yaw = math.degrees(msg.yaw) % 360
    target_yaw = (initial_yaw + degrees) % 360
    print(f"[ROTATE] Yaw iniziale: {initial_yaw:.2f}°, target: {target_yaw:.2f}°")

    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
        if not msg:
            continue

        current_yaw = math.degrees(msg.yaw) % 360
        delta = (target_yaw - current_yaw + 180) % 360 - 180

        if abs(delta) <= yaw_tolerance_deg:
            break

        rc_yaw = map_angle_to_rc_speed(delta)
        sub.send_rc(yaw=rc_yaw)

        print(f"[ROTATE] Yaw attuale: {current_yaw:.2f}°, Δ: {delta:.2f}°, RC: {rc_yaw}")
        time.sleep(0.05)

    sub.clear_motion()
    print("[ROTATE] Rotazione completata.")


def square_path(side_duration=5, motor_power=1700):
    """
    Esegue un percorso a quadrato utilizzando il controllo di yaw (orientamento) e profondità.

    Il robot:
    1. Registra la profondità iniziale leggendo un messaggio di pressione.
    2. Esegue un movimento in avanti per un lato del quadrato.
    3. Ruota di 90° per impostare la direzione del lato successivo.
    4. Ripristina la profondità iniziale dopo ogni lato per garantire stabilità verticale.
    5. Ripete il processo per 4 lati, completando così il quadrato.

    Parametri:
    - side_duration (int): Durata dell'avanzamento su ciascun lato (in secondi).
    - motor_power (int): Potenza dei motori durante l'avanzamento.
    """
    msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=1)
    initial_depth = depth_conversion(msg.press_abs)

    print("Inizio percorso quadrato...\n")
    for i in range(4):
        forward_with_yaw_and_depth_hold(duration_sec=side_duration, motor_power=motor_power)
        rotate_by(90)
        depth_movement(initial_depth)

    print("Percorso completato.")


def triangle_path(side_duration=5, motor_power=1700):
    """
    Esegue un percorso a triangolo equilatero utilizzando il controllo di yaw (orientamento) e profondità.

    Il robot:
    1. Avanza in linea retta per un tempo specificato (simulando il lato di un triangolo).
    2. Ruota di 120° per formare un angolo interno di un triangolo equilatero.
    3. Ripete il processo tre volte per completare il triangolo.

    Parametri:
    - side_duration (int): Durata dell'avanzamento su ciascun lato del triangolo (in secondi).
    - motor_power (int): Potenza dei motori durante l'avanzamento.
    """
    msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=1)
    initial_depth = depth_conversion(msg.press_abs)

    print("Inizio percorso triangolare...\n")
    for i in range(3):
        forward_with_yaw_and_depth_hold(duration_sec=side_duration, motor_power=motor_power)
        rotate_by(120)
        depth_movement(initial_depth)
    
    print("Percorso completato.")


def complex_path(straight_duration_sec=5, target_depth=0.5, motor_power=1700, lapsNumber = 1):
    """
    Esegue un percorso complesso composto da più giri (laps), utile per test di navigazione automatica
    in un ambiente controllato.

    Per ogni giro, il robot:
    1. Avanza in linea retta mantenendo profondità e assetto.
    2. Scende alla profondità specificata (target_depth).
    3. Ruota di 180°.
    4. Torna indietro mantenendo la stessa profondità.
    5. Risale a una profondità predefinita (0.145 m).
    6. Ruota di nuovo di 180° per prepararsi al giro successivo.

    Parametri:
    - straight_duration_sec (int): Durata dell'avanzamento/ritorno in secondi.
    - target_depth (float): Profondità alla quale scendere durante il giro.
    - motor_power (int): Potenza del motore durante l'avanzamento.
    - lapsNumber (int): Numero di giri da eseguire.
    """
    print("Inizio percorso complesso...\n")
    for i in range(lapsNumber):
        print(f"Avanzamento")
        forward_with_yaw_and_depth_hold(duration_sec=straight_duration_sec, motor_power=motor_power)
        print(f"Discesa")
        depth_movement(target_depth)
        print(f"Rotazione di 180°")
        rotate_by(180)
        print(f"Ritorno")
        forward_with_yaw_and_depth_hold(duration_sec=straight_duration_sec, motor_power=1700)
        print(f"Ascesa")
        depth_movement(0.145)
        print(f"Rotazione di 180°")
        rotate_by(180)
        print(f"Lap {i} completato")

    print("Percorso completato.")


def start_logging_yaw_depth(sub, durata_sec=10, filename="log_yaw_depth.csv", interval=0.25):
    """
    Registra yaw e profondità in un file CSV mentre il drone si muove.
    Può essere avviata prima del movimento ed eseguita in parallelo in un thread.

    Durante la registrazione, la funzione:
    1. Riceve messaggi ATTITUDE per ottenere l'angolo di yaw.
    2. Riceve messaggi SCALED_PRESSURE2 per ottenere la pressione assoluta (usata come profondità).
    3. Salva timestamp, yaw (in gradi) e profondità (in metri) nel file CSV a intervalli regolari.

    Parametri:
    - sub: istanza del sistema MAVLink o drone per ricevere i messaggi
    - durata_sec (float): durata totale della registrazione in secondi (default 10)
    - filename (str): nome del file CSV di output (default "log_yaw_depth.csv")
    - interval (float): intervallo tra le misurazioni in secondi (default 0.25)
    """
    def logger():
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["timestamp", "yaw_deg", "depth_m"])
            
            start_time = time.time() 
            while time.time() - start_time < durata_sec:
                timestamp = time.time()
                yaw_deg = None
                depth_m = None

                attitude_msg = sub.recv_match(type='ATTITUDE', blocking=True, timeout=0.2)
                depth_msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=0.2)

                if attitude_msg:
                    yaw_deg = math.degrees(attitude_msg.yaw)
                if depth_msg:
                    depth_m = depth_msg.press_abs

                if yaw_deg is not None and depth_m is not None:
                    writer.writerow([timestamp, yaw_deg, depth_m])
                    print(f"[LOG] Yaw: {yaw_deg:.2f}°, Depth: {depth_m:.2f} m")

                time.sleep(interval)

        print(f"[LOG] Salvataggio completato: {filename}")

    # Avvia il logger in un thread parallelo
    logging_thread = threading.Thread(target=logger)
    logging_thread.start()
    return logging_thread  # se vuoi fare join()


def depth_test():
    """
    Esegue un test di lettura della profondità per 30 secondi.

    Durante l'esecuzione, la funzione legge ripetutamente i messaggi MAVLink 
    di tipo 'SCALED_PRESSURE2' dal sensore di pressione del veicolo. 
    Per ogni lettura, stampa:
      - la pressione assoluta in mbar,
      - la temperatura in °C,
      - la profondità stimata in metri (convertita tramite `depth_conversion()`).
    """
    start_time = time.time()
    while time.time() - start_time < 30:
        msg = sub.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=1)
        if msg:
            print(f"Pressione assoluta: {msg.press_abs:.2f} mbar")
            print(f"Temperatura: {msg.temperature / 100.0:.2f} °C")
            profondita = depth_conversion(msg.press_abs)
            print(f"Profondità stimata: {profondita:.5f} m")


def main():
    global sub
    mavlink = "udpin:0.0.0.0:14550"
    client = mavlink   

    print('Inizio programma\n')
    sub = rc.Autopilot(mavlink, client=client)
    sub.set_mode('MANUAL')
    sub.arm()
    time.sleep(1)

    #depth_test()

    #forward_with_yaw_and_depth_hold(duration_sec=2, motor_power=1800)

    #rotate_by(90)

    #depth_movement(target_depth_m=0.2)

    #triangle_path(side_duration=3)
    #square_path(side_duration=3, motor_power=1700)

    #complex_path(straight_duration_sec=5, target_depth=0.5, lapsNumber = 1)

    #log_thread = start_logging_yaw_depth(sub, durata_sec=10, filename="log_forward_sendrc2.csv")

    # Movimento semplice
    sub.send_rc(forward=1800)
    #time.sleep(10)
    sub.status_loop(10)
    #sub.clear_motion()
    #forward_with_yaw_and_depth_hold(duration_sec=10, motor_power=1800)


    # Attendi fine logging
    #log_thread.join()


    #sub.clear_motion()
    #sub.disarm()


if __name__ == "__main__":
    main()
