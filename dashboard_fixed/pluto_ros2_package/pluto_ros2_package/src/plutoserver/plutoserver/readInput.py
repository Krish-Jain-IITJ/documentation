from pynput import keyboard

def on_press(key):
    try:
        char = key.char
        print(f"Pressed key: {char}")
    except AttributeError:
        print(f"Pressed key: {key}")

# Collect events until released
with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
