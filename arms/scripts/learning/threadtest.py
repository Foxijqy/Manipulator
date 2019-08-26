#!/usr/bin/env python

import threading
import time

class StoppableThread1(threading.Thread):
	def __init__(self):
		super(StoppableThread1, self).__init__()
		self._stop_event = threading.Event()
	
	def stop(self):
		self._stop_event.set()
	
	def stopped(self):	
		return self._stop_event.is_set()

	def run(self):
		while True:
			print "thread-1 running"
			time.sleep(1)

class StoppableThread2(threading.Thread):
	def __init__(self):
		super(StoppableThread2, self).__init__()
		self._stop_event = threading.Event()
	
	def stop(self):
		self._stop_event.set()
	
	def stopped(self):	
		return self._stop_event.is_set()

	def run(self):
		while True:
			print "thread-2 running"
			time.sleep(1)

def main():
	thread1 = StoppableThread1()
	thread2 = StoppableThread2()
	
	print thread1.stopped()
	thread1.start()
	thread2.start()
	time.sleep(5)
	thread1.stop()
	print thread1.stopped()
	time.sleep(5)
	thread1.join()
	print thread1.stopped()
	

'''
if __name__ == '__main__':
	try:
		main()
	except:
		pass

'''
main()
