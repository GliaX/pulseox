class Finger_clampAssembly < CrystalScad::Assembly
		
	# Assemblies are used to show how different parts interact on your design.
 
	# Skip generation of the 'output' method for this assembly.
	# (will still generate 'show')
	skip :output

	def part(show)
		# Create a test cube
		cube = Clamp.new
		# always make sure the lowest statement always returns the object that you're working on
		res
	end

end
