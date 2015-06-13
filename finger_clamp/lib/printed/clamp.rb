class Clamp < CrystalScad::Printed
	def initialize()
		@x = 28
		@y = 25
		@z = 2.5
		@top_z = 10
		@led_pcb = LedPcb.new.rotate(y:-35).translate(x:15,y:2,z:@z)
		@sensor_pcb = SensorPcb.new.rotate(z:90).rotate(y:35).translate(x:10,y:3.5,z:@z)
		@finger = Finger.new.rotate(x:-90).translate(x:12,y:-25,z:@z+8)
	end
	
	def part(show)
		res = cube(@x,y,@z)		
	
		res += right_triangle(15,10.5).linear_extrude(h:@y).rotate(x:90).translate(y:@y)
		res -= @sensor_pcb.output

		res += right_triangle(18,12.5).mirror(x:1).linear_extrude(h:@y).rotate(x:90).translate(y:@y).translate(x:28)
		res -= @led_pcb.output

		if show		
			res += @led_pcb.show	
			res += @sensor_pcb.show
			res += @finger.show
			res += top_piece.mirror(z:1).translate(z:top_z*2)
		else
			res += top_piece.translate(x:@x+1)
		end

		
		res
	end

	view :top_piece

	def top_piece
		res = cube(@x,y,@top_z)		
		res -= cylinder(d:14,h:70).scale(x:1.5,y:1).rotate(x:-90).translate(x:14,y:-1,z:12)		
	end

	def right_triangle(b,c)
		points = []
		points << [0,0]
		points << [b,0]
		points << [0,c]

		polygon(points:points)
	end

end	
