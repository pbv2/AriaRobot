"""
retorna dicionarios com coordenadas das consultas no mapa de caminho filepath
/usr/local/Aria/maps/office.map no osboxes
"""
def get_map_min_pos(filepath):
    
    file = open(filepath)
    read = file.readline()
    while 'LineMinPos' not in read:
        read = file.readline()
        if not read: # evita um loop inf se mapa nao tiver informacao de posicao min
            return None
    read = read.split()
    try: 
        min_pos = {'x': float(read[1]), 'y': float(read[2])}
        return min_pos
    except Exception as e:
        print e

def get_robot_home(filepath):
    file = open(filepath)
    read = file.readline()
    while 'RobotHome' not in read:
        read = file.readline()
        if not read: # evita um loop inf se mapa nao tiver informacao de robot home
            return None
    read = read.split()
    try:
        robot_home = {'x': float(read[2]), 'y': float(read[3]), 't': float(read[4])}
        return robot_home
    except Exception as e:
        print e

def get_map_max_pos(filepath):
    file = open(filepath)
    read = file.readline()
    while 'LineMaxPos' not in read:
        read = file.readline()
        if not read: # evita um loop inf se mapa nao tiver informacao de posicao min
            return None
    read = read.split()
    try: 
        max_pos = {'x': float(read[1]), 'y': float(read[2])}
        return max_pos
    except Exception as e:
        print e
