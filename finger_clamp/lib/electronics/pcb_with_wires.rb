class PcbWithWires < CrystalScad::Assembly
	

	def output_block	
		height = 4
		res = cube(x:4,y:8,z:height).translate(z:-height)
	end
	
	def output_wires
		res += long_slot(d:1.5,l:5,h:100).rotate(z:-90).rotate(x:-90).translate(x:2,z:-3)
	end
		
	
end
