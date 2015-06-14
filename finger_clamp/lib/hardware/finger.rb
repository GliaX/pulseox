class Finger < CrystalScad::Assembly

	skip :output

	def part(show)
		# index finger measurements	
		# width x fingernail height (height) 
		# Jenn 15.2x10.3 (12)
		#	Kliment 16x11.8 (12)
		# Spencer 13.5x9.4 (12.4)
		# Renee 12.7x7.68 (10.5)
		# Tarek 15.5x10.7 (12.7)
		#	Tareks dad 15.8x10.6 (12.6)


		cylinder(d:10,h:40).color("LightSalmon")
	end


end
