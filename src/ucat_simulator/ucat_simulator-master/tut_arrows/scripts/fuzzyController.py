import numpy as np

class FuzzyLogicController:
	"""
	A Fuzzy Logic controller for speed control

	...

	Attributes
	----------
	uod1 : [float, float, float]
		Universe of discourse of first input (distance)
	uod2 : [string, string, string]
		Universe of discourse of second input (traffic lights)
	uodOutput : [float, float, float]
		Universe of discourse for the output (speed)
	rulesTable : [[string, string, string],
				  [string, string, string],
				  [string, string, string]]
		rules table based on the distance and the traffic lights linguistic values

	Methods
	-------
	fuzzify(distance, trafficLight):
		Fuzzification: This basically transforms the inputs from crisp values to linguistic values
		Returns the membership degree of the inputs (distance and traffic lights) to their respective membership functions
	computeRules(MU):
		Inference mechanism
		Returns the membership degree values for the output
	defuzzify(muSpeed):
		Defuzzification
		Returns the output by transforming the output from linguistic to crips value (using the Center of Gravity method)
	control(distance, trafficLight):
		This function applies the fuzzy logic process to compute the required speed
	"""
	def __init__(self, universeOfDiscourse1, universeOfDiscourse2, universeOfDiscourseOutput, rulesTable):
		self.uod1 = universeOfDiscourse1 # will use shorter names 
		self.uod2 = universeOfDiscourse2
		self.uodOutput = universeOfDiscourseOutput
		self.rulesTable = rulesTable


	def fuzzify(self, x, delta):
		# X fuzzification
		if ( x <= self.uod1[0] ): # If the distance is "very close"
			mu_x_neg = 1
			mu_x_zero = 0
			mu_x_pos = 0

		elif ( x > self.uod1[0] and x <= self.uod1[1] ):
			mu_x_neg = (x - self.uod1[1]) / (self.uod1[0] - self.uod1[1])
			mu_x_zero = (x - self.uod1[0]) / (self.uod1[1] - self.uod1[0])
			mu_x_pos = 0

		elif ( x > self.uod1[1] and x <= self.uod1[2] ):
			mu_x_neg = 0
			mu_x_zero = 1
			mu_x_pos = 0

		elif ( x > self.uod1[2] and x <= self.uod1[3] ):
			mu_x_neg = 0
			mu_x_zero = (x - self.uod1[3]) / (self.uod1[2] - self.uod1[3])
			mu_x_pos = (x - self.uod1[2]) / (self.uod1[3] - self.uod1[2])

		else:
			mu_x_neg = 0
			mu_x_zero = 0
			mu_x_pos = 1

		# Delta fuzzification
		if ( delta <= self.uod2[0] ):
			mu_delta_neg = 1
			mu_delta_zero = 0
			mu_delta_pos = 0

		elif ( delta > self.uod2[0] and delta <= self.uod2[1] ):
			mu_delta_neg = (delta - self.uod2[1]) / (self.uod2[0] - self.uod2[1])
			mu_delta_zero = (delta - self.uod2[0]) / (self.uod2[1] - self.uod2[0])
			mu_delta_pos = 0

		elif ( delta > self.uod2[1] and delta <= self.uod2[2] ):
			mu_delta_neg = 0
			mu_delta_zero = 1
			mu_delta_pos = 0

		elif ( delta > self.uod2[2] and delta <= self.uod2[3] ):
			mu_delta_neg = 0
			mu_delta_zero = (delta - self.uod2[3]) / (self.uod2[2] - self.uod2[3])
			mu_delta_pos = (delta - self.uod2[2]) / (self.uod2[3] - self.uod2[2])

		else:
			mu_delta_neg = 0
			mu_delta_zero = 0
			mu_delta_pos = 1
		            
		# print('membership =', np.array([mu_x_neg, mu_x_zero, mu_x_pos, mu_delta_neg, mu_delta_zero, mu_delta_pos]) )
		return np.array([mu_x_neg, mu_x_zero, mu_x_pos, mu_delta_neg, mu_delta_zero, mu_delta_pos])

	def computeRules(self, MU):
		#Rules computing
		mu_R1 = min( MU[3], MU[0] )
		mu_R2 = min( MU[3], MU[1] )
		mu_R3 = min( MU[3], MU[2] )
		mu_R4 = min( MU[4], MU[0] )
		mu_R5 = min( MU[4], MU[1] )
		mu_R6 = min( MU[4], MU[2] )
		mu_R7 = min( MU[5], MU[0] )
		mu_R8 = min( MU[5], MU[1] )
		mu_R9 = min( MU[5], MU[2] )

		mu_table = np.array([[mu_R1, mu_R2, mu_R3],
				     [mu_R4, mu_R5, mu_R6],
				     [mu_R7, mu_R8, mu_R9]])
		
		#Firing rules
		mu_neg = max( mu_table[np.where(self.rulesTable == 'neg')]);
		mu_stop = max( mu_table[np.where(self.rulesTable == 'stop')]);
		mu_pos = max( mu_table[np.where(self.rulesTable == 'pos')]);

		# print('membership output =', [mu_neg, mu_stop, mu_pos] )    
		return np.array ( [mu_neg, mu_stop, mu_pos] )    

	def defuzzify(self,muSpeed):
		speed = (self.uodOutput[0] * muSpeed[0] + self.uodOutput[1] * muSpeed[1] + self.uodOutput[2] * muSpeed[2]) / ( muSpeed[0] + muSpeed[1] + muSpeed[2] )
		# print('speed output =', speed )    

		return speed

	def control(self, val, deriv):  
		MU = self.fuzzify(val, deriv)
		muSpeed = self.computeRules(MU)
		speed = self.defuzzify(muSpeed)

		return speed
