class SensorPcb < PcbWithWires

	def initialize
		@x = 17.1
		@y = 11.5
		@z = 1.7


		@sensor_x=0.8
		@sensor_y=0.8
		@sensor_z=3.15 # total height from pcb bottom

		@cap_x = 1
		@cap_y = 1

		@cap_positions_x = 10.7
		@cap_positions_y = 7.3
				
		@sensor_positions_x = 2.6
		@sensor_positions_y = 3.6

		@pad_height = 1.72
		@layer_height = 0.2	

		# fixme: position in the center
		@conn_positions_x = 13.75	
		@conn_positions_y1 = 2.2
		@conn_positions_y2 = 4.85
		@conn_positions_y3 = 7.5

		@output_margin = 0.3

		@color = "DarkGreen"
	end

	def part(show)
		if show
			margin = 0
		else
			margin = @output_margin
		end
		res = cube(@x+margin,@y+margin,@z).translate(x:-margin/2.0,y:-margin/2.0)
		
		res = colorize(res)
		
		res += sensor(show).translate(x:@sensor_positions_x,y:@sensor_positions_y)			
		res += cap(show).translate(x:@cap_positions_x,y:@cap_positions_y)

		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y1)		
		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y2)		
		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y3)		

		if !show
			res += output_block.translate(x:@conn_positions_x-2.5,y:2)
			res += output_wires.rotate(z:-90).translate(x:@conn_positions_x-2.5,y:5,z:1)
			
		end

		res
	end
	
	def connector_pad(show)
		cylinder(d:1,h:@pad_height).scale(x:3,y:1.74).color("silver")
	end

	def sensor_pad
		cube(@sensor_x,@sensor_y,@pad_height).color("silver")	
	end

	def sensor(show)
		res = sensor_pad
		res += sensor_pad.translate(x:3)
		res += sensor_pad.translate(x:3,y:1.5)
		res += sensor_pad.translate(y:1.5)

		if show	
			res += cube(3.4,2.6,@sensor_z-@pad_height).translate(x:0.2,z:@pad_height).color("Linen")
		end
		res
	end

	def cap(show)
		res = cap_pad
		res += cap_pad.translate(y:1.7)		
	end

	def cap_pad
		res = cube(x:1,y:1,z:@pad_height).color("silver")
	end

end
