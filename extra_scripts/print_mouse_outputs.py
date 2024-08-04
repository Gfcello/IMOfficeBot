from pynput import mouse

def on_move(x, y):
    print('Pointer moved to {0}'.format(
        (x, y)))

listener = mouse.Listener(
    on_move=on_move)
listener.start()

