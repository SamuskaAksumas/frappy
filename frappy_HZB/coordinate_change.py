import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def euler_to_rotation_matrix(xr, yr, zr):
    """
    Wandelt Euler-Winkel (xyz) in eine Rotationsmatrix um.
    """
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(xr), -np.sin(xr)],
                   [0, np.sin(xr), np.cos(xr)]])
    
    Ry = np.array([[np.cos(yr), 0, np.sin(yr)],
                   [0, 1, 0],
                   [-np.sin(yr), 0, np.cos(yr)]])
    
    Rz = np.array([[np.cos(zr), -np.sin(zr), 0],
                   [np.sin(zr), np.cos(zr), 0],
                   [0, 0, 1]])
    
    # Gesamtrotationsmatrix
    return Rz @ Ry @ Rx

def transform_point_to_global(local_point, local_system):
    """
    Transformiert einen Punkt mit Position und Orientierung aus einem lokalen Koordinatensystem in das globale System.
    
    :param local_point: Koordinaten des Punkts im lokalen System (x', y', z', xr', yr', zr') in Metern und Radiant.
    :param local_system: Position und Orientierung des lokalen Systems im globalen System (x, y, z, xr, yr, zr) in Metern und Radiant.
    :return: Transformierte Koordinaten und Orientierung im globalen System (x, y, z, xr, yr, zr).
    """
    # Extrahiere Position und Rotation des lokalen Systems
    origin = np.array(local_system[:3])  # (x, y, z)
    system_rotation_angles = np.array(local_system[3:])  # (xr, yr, zr) in Radiant

    # Extrahiere Position und Rotation des Punkts im lokalen System
    point_position = np.array(local_point[:3])  # (x', y', z')
    point_rotation_angles = np.array(local_point[3:])  # (xr', yr', zr') in Radiant

    # Berechne die Rotationsmatrix des lokalen Systems
    system_rotation_matrix = euler_to_rotation_matrix(*system_rotation_angles)

    # Transformiere die Position des Punkts ins globale System
    rotated_position = system_rotation_matrix @ point_position
    global_position = rotated_position + origin

    # Kombiniere die Rotationen von Punkt und System
    point_rotation_matrix = euler_to_rotation_matrix(*point_rotation_angles)
    global_rotation_matrix = system_rotation_matrix @ point_rotation_matrix

    # Extrahiere die globalen Euler-Winkel aus der kombinierten Rotationsmatrix
    global_rotation_angles = np.arctan2([global_rotation_matrix[2, 1], global_rotation_matrix[0, 2], global_rotation_matrix[1, 0]], 
                                        [global_rotation_matrix[2, 2], global_rotation_matrix[0, 0], global_rotation_matrix[1, 1]])

    return np.hstack((global_position, global_rotation_angles))

if __name__ == '__main__':

    # Beispielwerte
    local_point = [1.0, 0.0, 0.0, 0.174533, 0.349066, 0.523599]  # x', y', z', xr', yr', zr' (in Radiant)
    local_system = [1.0, 2.0, 3.0, 0.523599, 0.785398, 1.047198]  # x, y, z, xr, yr, zr (in Radiant)

    # Transformiere den Punkt ins globale System
    global_point = transform_point_to_global(local_point, local_system)
    print("Globaler Punkt und Orientierung:", global_point)

    # Visualisierung
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot des globalen Koordinatensystems
    ax.quiver(0, 0, 0, 1, 0, 0, color='k', linewidth=2)
    ax.quiver(0, 0, 0, 0, 1, 0, color='k', linewidth=2)
    ax.quiver(0, 0, 0, 0, 0, 1, color='k', linewidth=2)
    ax.text(1.1, 0, 0, "X", color='k')
    ax.text(0, 1.1, 0, "Y", color='k')
    ax.text(0, 0, 1.1, "Z", color='k')

    # Plot des lokalen Koordinatensystems
    local_origin = np.array(local_system[:3])
    local_rotation_matrix = euler_to_rotation_matrix(*local_system[3:])
    local_axes = local_rotation_matrix @ np.eye(3)  # Lokale Koordinatenachsen

    ax.quiver(*local_origin, *local_axes[:, 0], color='b', linewidth=2)
    ax.quiver(*local_origin, *local_axes[:, 1], color='b', linewidth=2)
    ax.quiver(*local_origin, *local_axes[:, 2], color='b', linewidth=2)
    ax.text(*local_origin + local_axes[:, 0] * 1.1, "X'", color='b')
    ax.text(*local_origin + local_axes[:, 1] * 1.1, "Y'", color='b')
    ax.text(*local_origin + local_axes[:, 2] * 1.1, "Z'", color='b')

    # Punkt im lokalen Koordinatensystem
    ax.scatter(*local_origin, color='blue', s=100, label='Lokales Ursprungssystem')
    local_point_position = local_origin + local_rotation_matrix @ np.array(local_point[:3])
    ax.scatter(*local_point_position, color='red', s=100, label='Punkt im lokalen System')

    # Transformierter Punkt im globalen Koordinatensystem
    ax.scatter(*global_point[:3], color='green', s=100, label='Punkt im globalen System')

    # Achsen und Ansicht
    ax.set_xlim([-1, 4])
    ax.set_ylim([0, 5])
    ax.set_zlim([0, 5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Transformation eines Punkts zwischen Koordinatensystemen (mit Rotationsmatrix)")
    ax.legend()

    plt.show()
