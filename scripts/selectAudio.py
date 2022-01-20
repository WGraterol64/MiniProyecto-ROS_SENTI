from os import listdir
from pathlib import Path
from os.path import isfile
from simple_term_menu import TerminalMenu

root = '/usr/src/pepper_sim_ws/src/pepper_robot/pepper_sensors_py/nodes/sounds/'
soundFile = '/usr/src/pepper_sim_ws/src/pepper_robot/pepper_sensors_py/nodes/soundfile.txt'

def listFiles(dir: str):
	""" Lista los archivos y directorios desde un directorio. """
	# Obtenemos todos los archivos, los directorios les agregamos '/' al final y los
	# ordenamos alfabeticamente.
	files = [f + ('/' if not isfile(dir + f) else '') for f in listdir(dir)]
	files.sort()
	return files

def selectFile(dir: str) -> str:
	""" 
		Se muestra un meno para que el usuario elija un archivo desde el directorio
		actual. 
	"""
	files = listFiles(dir)
	menu = TerminalMenu(files)
	index = menu.show()
	return files[index]

def main():
	path = str(Path(root).absolute()) + '/'
	found = False
	try:
		# Nos movemos a traves de los directorios hasta que se elija un archivo regular.
		while True:
			file = selectFile(path)
			if file == '..':
				path = str(Path(path).parent.absolute()) + '/'
			else:
				path = str(Path(path + file).absolute())

			if isfile(path): 
				found = True
				break 
			else: path += '/'

		# Escribimos el archivo seleccionado en el soundfile
		if found:
			with open(soundFile, 'w') as f:
				f.write(path)
	except:
		pass

main()