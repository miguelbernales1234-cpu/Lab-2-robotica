from controller import Robot, DistanceSensor, PositionSensor
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())
ts = timestep / 1000.0

RADIO_RUEDA = 0.0205

motor_izquierdo = robot.getDevice('left wheel motor')
motor_derecho = robot.getDevice('right wheel motor')
motor_izquierdo.setPosition(float('inf'))
motor_derecho.setPosition(float('inf'))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)
sensores_distancia = {}
nombres_sensores = {
    'frontal_der': 'ps0',
    'frontal_izq': 'ps7',
    'lateral_der': 'ps2',
    'lateral_izq': 'ps5'
}

for clave, nombre_webots in nombres_sensores.items():
    sensores_distancia[clave] = robot.getDevice(nombre_webots)
    sensores_distancia[clave].enable(timestep)

encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(timestep)
encoder_derecho.enable(timestep)

last_enc_izq = 0.0
last_enc_der = 0.0
alpha = 0.2  
valor_filtrado_frontal = 0.4  
d_estimada = 0.4   
P = 1.0            
Q = 0.0001         
R_noise = 0.02

direccion_giro = None 
historial_tiempo = []
historial_crudo = []
historial_filtrado = []
historial_kalman = []

def promediar_sensores_en_metros(val_izq, val_der):
    
    val_max = max(val_izq, val_der)
    if val_max < 100:
        return 0.40 
    
    distancia = 15.0 / (val_max ** 0.8)
    return min(max(distancia, 0.02), 0.40)

while robot.step(timestep) != -1:
    tiempo_actual = robot.getTime()
    
    raw_f_der = sensores_distancia['frontal_der'].getValue()
    raw_f_izq = sensores_distancia['frontal_izq'].getValue()
    raw_l_der = sensores_distancia['lateral_der'].getValue()
    raw_l_izq = sensores_distancia['lateral_izq'].getValue()
    enc_izq = encoder_izquierdo.getValue()
    enc_der = encoder_derecho.getValue()
    
    distancia_frontal_cruda = promediar_sensores_en_metros(raw_f_izq, raw_f_der)
    
    delta_th_izq = enc_izq - last_enc_izq
    delta_th_der = enc_der - last_enc_der
    despl_izq = RADIO_RUEDA * delta_th_izq
    despl_der = RADIO_RUEDA * delta_th_der
    avance_robot = (despl_izq + despl_der) / 2.0
    
    last_enc_izq = enc_izq
    last_enc_der = enc_der
    valor_filtrado_frontal = (alpha * distancia_frontal_cruda) + ((1.0 - alpha) * valor_filtrado_frontal)
   
    delta_d = -avance_robot
    d_predicha = d_estimada + delta_d 
    P_predicha = P + Q 
    
    z_k = distancia_frontal_cruda 
    K = P_predicha / (P_predicha + R_noise)            
    d_estimada = d_predicha + K * (z_k - d_predicha)   
    P = (1.0 - K) * P_predicha                         
    
    historial_tiempo.append(tiempo_actual)
    historial_crudo.append(distancia_frontal_cruda)
    historial_filtrado.append(valor_filtrado_frontal)
    historial_kalman.append(d_estimada)
    
    UMBRAL_SEGURIDAD = 0.16 
    
    if d_estimada > UMBRAL_SEGURIDAD and direccion_giro is None:
        v_izq = 4.0
        v_der = 4.0
    else:
        if direccion_giro is None:
            if raw_l_izq > raw_l_der:
                direccion_giro = 'DERECHA'  
            else:
                direccion_giro = 'IZQUIERDA' 
        
        if direccion_giro == 'DERECHA':
            v_izq = 2.0
            v_der = -2.0
        else:
            v_izq = -2.0
            v_der = 2.0  
        
        if d_estimada > (UMBRAL_SEGURIDAD + 0.04): 
            direccion_giro = None 
            
    motor_izquierdo.setVelocity(v_izq)
    motor_derecho.setVelocity(v_der)
    
    if len(historial_tiempo) % 15 == 0:
        print(f"[T: {tiempo_actual:.2f}s] Modo: {direccion_giro or 'RECTO'} | Crudo: {distancia_frontal_cruda:.3f}m | KALMAN FUSION: {d_estimada:.3f}m")
       