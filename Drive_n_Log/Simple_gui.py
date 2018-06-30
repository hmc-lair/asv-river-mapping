from Simple_graphics import *
from Simple_controller import *
def main():
	controller = Simple_controller()
	graphics = Simple_graphics(controller)

	if controller.start_connection() == False:
		# print('ok')
		return False

	print('Receiving data...')

	while True:
		try:
			# update graphics, but stop the thread if user stopped the gui
			if not graphics.update():
				break
			continue
		except(KeyboardInterrupt, SystemExit):
			print("Shutting sytem down...")

			raise

if __name__ == '__main__':
	main()