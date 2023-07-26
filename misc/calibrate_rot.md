Calibration code: 


Input this in ft.py: 

in _on_press():
	
 elif k == 'r':
 	self.recording = not self.recording
 	for remaining in range(5, 0, -1):
 		sys.stdout.flush()
 		sys.stdout.write("\r")
 		sys.stdout.write("{:2d} seconds remaining . . . .".format(remaining)) 
 		time.sleep(1)
	self.strt = time.time()

in loop():

if self.recording:
	time_elapsed = time.time() - self.strt
	if time_elapsed <= 5:
		self.rot.append(r)
		print('.')
	else:
		if self.rot:
			print('Mean:', np.mean(self.rot, axis = 0))
			writer.writerow(np.mean(self.rot, axis = 0))
			csvf.close()
			self.rot = []
			self.recording = not self.recording
			self.strt = 0
			print('\nRecording Stopped!')
