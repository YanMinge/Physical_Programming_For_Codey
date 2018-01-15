class XYImage:
	def __init__(self, image, x = 0, y = 0, center = None, angle = 0):
		self.image = image
		self.x = x
		self.y = y
		self.center = (x/2, y/2) if center is None else center
		self.angle = angle