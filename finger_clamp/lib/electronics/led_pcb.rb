class LedPcb < PcbWithWires

	def initialize
		@x = 14.2
		@y = 11.5
		@z = 1.7
		@led_x=0.9
		@led_y=1.7
		@led_z=2.5
		@led_width = 2.1
		@pad_height = 1.72
		@layer_height = 0.2	

		@led_positions_x = 1.3
		@led_positions_y1 = 4.3
		@led_positions_y2 = 6.2

		@conn_positions_x = 10.75	
		@conn_positions_y1 = 3.5
		@conn_positions_y2 = 6.4
		@conn_positions_y3 = 8.85
	
		# wires are ~1.3 thick

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
		
		res += led(show).translate(x:@led_positions_x,y:@led_positions_y1)			
		res += led(show).translate(x:@led_positions_x,y:@led_positions_y2)			

		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y1)		
		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y2)		
		res += connector_pad(show).translate(x:@conn_positions_x,y:@conn_positions_y3)		

		if !show
			res += output_block.translate(x:@conn_positions_x-2,y:2)
			res += output_wires.translate(x:@conn_positions_x-2,y:2)

		end

		res
	end
	
	def connector_pad(show)
		cylinder(d:1,h:@pad_height).scale(x:3,y:1.74).color("silver")
	end

	def led_pad
		cube(@led_x,@led_y,@pad_height).color("silver")	
	end

	def led(show)
		res = led_pad
		res += led_pad.translate(x:@led_width-@led_x)
		res += cube(1.7,1.6,@led_height-@pad_height).translate(x:0.2,z:@pad_height)
	end

end
