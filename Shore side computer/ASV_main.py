from ASV_environment import *
from ASV_graphics import *

a = ASV_environment()

for i in range(10):
	a.xbee.tx(dest_addr = '\x00\x0A', data = "hello")

# def main():
# 	a = ASV_environment()

# main()