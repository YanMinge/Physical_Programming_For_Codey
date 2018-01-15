import codey
import rocky
codey.face('00001020402012020212204020100000',0.5)

def on_button_callback():
	rocky.forward(50,0.5)
	rocky.forward(50,0.5)
	rocky.forward(50,0.5)
	rocky.turn_right_angle(90)
	rocky.forward(50,0.5)
	rocky.forward(50,0.5)
	codey.face('00001020402012020212204020100000',0.5)
	codey.say('cat.wav')

codey.on_button('A', on_button_callback)
