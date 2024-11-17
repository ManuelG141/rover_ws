#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

# Parámetro para detectar saltos de ángulo
ANGLE_JUMP_THRESHOLD = 10.0
DISTANCE_JUMP_THRESHOLD = 0.025  # Umbral de salto en distancia (metros)
DISTANCE_MERGE_THRESHOLD = 0.1# En metros (30 cm)

# Diccionario para llevar un registro de los marcadores activos
active_marker_ids = set()
current_marker_ids = set()


def merge_close_groups(angle_groups, range_groups):
    # Listas para los grupos combinados
    merged_angle_groups = []
    merged_range_groups = []

    # Comienza con el primer grupo
    current_angles = angle_groups[0]
    current_ranges = range_groups[0]

    for i in range(1, len(angle_groups)):
        # Calcula la distancia promedio del grupo actual
        dprom_current = np.mean(current_ranges)
        dprom_next = np.mean(range_groups[i])

        # Si la distancia entre los grupos es menor que el umbral, combínalos
        if abs(dprom_next - dprom_current) < DISTANCE_MERGE_THRESHOLD:
            current_angles.extend(angle_groups[i])
            current_ranges.extend(range_groups[i])
        else:
            # Si no están cerca, guarda el grupo actual y comienza uno nuevo
            merged_angle_groups.append(current_angles)
            merged_range_groups.append(current_ranges)
            current_angles = angle_groups[i]
            current_ranges = range_groups[i]

    # Agrega el último grupo combinado
    merged_angle_groups.append(current_angles)
    merged_range_groups.append(current_ranges)

    return merged_angle_groups, merged_range_groups

def calculate_adjusted_mean_angle(angles):
    # Si el rango de los ángulos cruza 0° (cuando la diferencia entre el mayor y menor ángulo es grande)
    if max(angles) - min(angles) > 180:
        # Ajusta los ángulos para que estén en un rango continuo
        adjusted_angles = [(angle + 360) if angle < 180 else angle for angle in angles]
        mean_angle = np.mean(adjusted_angles)
        # Asegúrate de devolver el promedio en el rango 0°-360°
        if mean_angle >= 360:
            mean_angle -= 360
    else:
        # Si no cruza 0°, simplemente calcula el promedio normalmente
        mean_angle = np.mean(angles)
    
    return mean_angle


# Crear el publicador para los marcadores de RViz
marker_pub = rospy.Publisher("/obstacle_markers", MarkerArray, queue_size=10)

def callback(data):
    # Variables de ángulo inicial y el incremento angular
    angle_min = data.angle_min
    angle_increment = data.angle_increment

    # Listas para almacenar los grupos de ángulos y distancias
    angle_groups = []
    range_groups = []
    current_angles = []
    current_ranges = []
    distances = []

    last_angle_deg = None
    last_range = None

    for i, range in enumerate(data.ranges):
        # Calcular el ángulo correspondiente al índice i
        angle = angle_min + i * angle_increment
        angle_deg = angle * 180 / 3.141592

        # Ajustar el rango de -180 a 180 grados a 0 a 360 grados
        if angle_deg < 0:
            angle_deg += 360

        if range < 1.2:  # Filtrar los puntos que estén a menos de x metro
            # Inicializar el primer ángulo y distancia
            if last_angle_deg is None:
                last_angle_deg = angle_deg
                last_range = range

            # Detectar salto en ángulo o en distancia
            angle_jump = abs(angle_deg - last_angle_deg) > ANGLE_JUMP_THRESHOLD
            distance_jump = abs(range - last_range) > DISTANCE_JUMP_THRESHOLD

            if angle_jump or distance_jump:  # Iniciar un nuevo grupo si hay un salto
                if current_angles:
                    angle_groups.append(current_angles)
                    range_groups.append(current_ranges)
                    current_angles = []
                    current_ranges = []

            # Añadir el ángulo y la distancia al grupo actual
            current_angles.append(angle_deg)
            current_ranges.append(range)

            # Actualizar el último ángulo y distancia
            last_angle_deg = angle_deg
            last_range = range

    if current_angles:
        angle_groups.append(current_angles)
        range_groups.append(current_ranges)

        angle_groups, range_groups = merge_close_groups(angle_groups, range_groups)


    # Mostrar la información de los grupos detectados
    text = "Obstáculos detectados:\n"
    if angle_groups:
        for idx, (angles, ranges) in enumerate(zip(angle_groups, range_groups)):
            dprom = np.mean(ranges)
            distances.append(round(dprom, 2))
            text += f"Grupo {idx + 1}: {round(angles[0], 2)}° - {round(angles[-1], 2)}° Distancia aprox: {round(dprom, 2)} m\n"
    else:
        text += "No se detectaron obstáculos."

    # Publicar la información en la consola
    rospy.loginfo(text)
    

    # Crear un MarkerArray para publicar en RViz
    marker_array = MarkerArray()
    marker_id=()
    active_marker_ids.clear()
    #markers_dict = {}
    
    for idx, (angles, ranges, distan) in enumerate(zip(angle_groups, range_groups,distances)):
        # Crear un marcador para cada grupo de obstáculos detectados
        marker_id = idx
        current_marker_ids.add(marker_id)

        marker = Marker()
        marker.header.frame_id = "laser"  # Cambia esto según el frame de tu LiDAR
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Calcular la posición promedio del grupo de obstáculos
        dprom = np.mean(ranges)
        angle_prom = calculate_adjusted_mean_angle(angles)
        x = dprom * np.cos(np.radians(angle_prom))
        y = dprom * np.sin(np.radians(angle_prom))

        # Configurar la posición y tamaño del marcador
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0  # Para 2D LiDAR
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        # Color del marcador
        if distan < 0.6:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6  # Alpha (transparencia)
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3  # Alpha (transparencia)

        marker_array.markers.append(marker)
        active_marker_ids.add(marker_id)

        #================== TEXTO  ==============
        # Crear un marcador de texto para mostrar el número de grupo
        text_marker = Marker()
        text_marker.header.frame_id = "laser"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "obstacle_numbers"
        text_marker.id = marker_id + 1000  # Evita conflictos con los IDs de los cubos
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        # Configurar la posición del marcador de texto (ligeramente encima del obstáculo)
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = 0.3  # Para que el texto esté un poco arriba del cubo
        text_marker.pose.orientation.w = 1.0

        # Configurar el tamaño y color del texto
        text_marker.scale.z = 0.1  # Tamaño del texto
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0  # Sin transparencia

        # Asignar el número de grupo como texto
        #text_marker.text = f"G{idx}:{distan}m / ({round(angles[0], 2)}° - {round(angles[-1], 2)}°)"
        text_marker.text = f"G{idx}:{distan}m"
        marker_array.markers.append(text_marker)

        #========================================

        #markers_dict[idx] = marker_id

    # Eliminar marcadores que ya no están activos
    for marker_id in current_marker_ids:
        if marker_id not in active_marker_ids:
            # Crear un marcador con acción DELETE para quitarlo de RViz
            delete_marker = Marker()
            delete_marker.header.frame_id = "laser"
            delete_marker.header.stamp = rospy.Time.now()
            delete_marker.ns = "obstacles"
            delete_marker.id = marker_id
            delete_marker.action = Marker.DELETE
            

            delete_text_marker = Marker()
            delete_text_marker.header.frame_id = "laser"
            delete_text_marker.header.stamp = rospy.Time.now()
            delete_text_marker.ns = "obstacle_numbers"
            delete_text_marker.id = marker_id + 1000  # Evita conflictos con los IDs de los cubos
            delete_text_marker.action = Marker.DELETE

            marker_array.markers.append(delete_marker)
            marker_array.markers.append(delete_text_marker)
    # Publicar el MarkerArray en ROS
    marker_pub.publish(marker_array)

    # Actualizar el conjunto de IDs activos
    
    current_marker_ids.update(active_marker_ids)

    # Imprimir los IDs activos para verificar
    print("Marcadores activos:", active_marker_ids)
    print("Current Markers Id: ", current_marker_ids)



def listener():
    rospy.init_node('lidar_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    # Iniciar el nodo de ROS
    listener()