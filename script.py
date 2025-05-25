import serial
from pynput import keyboard
import threading
import time

# Налаштування порту (змініть на ваш)
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Словник для зберігання стану кожної клавіші
key_states = {
    'up': '0',
    'down': '0',
    'left': '0',
    'right': '0'
}

# Змінна для збереження попереднього стану
previous_state = ''
lock = threading.Lock()
running = True
state_changed = threading.Event()

def get_current_state():
    """Повертає поточний стан всіх клавіш як рядок"""
    return key_states['up'] + key_states['down'] + key_states['left'] + key_states['right']

def on_press(key):
    try:
        with lock:
            changed = False
            if key == keyboard.Key.up and key_states['up'] != 'A':
                key_states['up'] = 'A'
                changed = True
            elif key == keyboard.Key.down and key_states['down'] != 'B':
                key_states['down'] = 'B'
                changed = True
            elif key == keyboard.Key.left and key_states['left'] != 'D':
                key_states['left'] = 'D'
                changed = True
            elif key == keyboard.Key.right and key_states['right'] != 'C':
                key_states['right'] = 'C'
                changed = True
            
            if changed:
                state_changed.set()
    except AttributeError:
        pass

def on_release(key):
    global running
    try:
        with lock:
            changed = False
            if key == keyboard.Key.up and key_states['up'] != '0':
                key_states['up'] = '0'
                changed = True
            elif key == keyboard.Key.down and key_states['down'] != '0':
                key_states['down'] = '0'
                changed = True
            elif key == keyboard.Key.left and key_states['left'] != '0':
                key_states['left'] = '0'
                changed = True
            elif key == keyboard.Key.right and key_states['right'] != '0':
                key_states['right'] = '0'
                changed = True
            elif key == keyboard.Key.esc:
                running = False
                state_changed.set()
                return False
            
            if changed:
                state_changed.set()
    except AttributeError:
        pass

def send_data():
    global previous_state
    
    while running:
        # Чекаємо на зміну стану
        state_changed.wait()
        state_changed.clear()
        
        if not running:
            break
            
        with lock:
            current_state = get_current_state()
            
            # Відправляємо тільки якщо стан змінився
            if current_state != previous_state:
                message = current_state + '\n'
                ser.write(message.encode())
                
                # Візуалізація
                display = message.strip().replace('0', '_')
                arrows = []
                if key_states['up'] == 'A': arrows.append('↑')
                if key_states['down'] == 'B': arrows.append('↓')
                if key_states['left'] == 'D': arrows.append('←')
                if key_states['right'] == 'C': arrows.append('→')
                
                arrow_display = ' '.join(arrows) if arrows else 'нічого'
                timestamp = time.strftime('%H:%M:%S')
                print(f"[{timestamp}] Зміна стану: [{display}] - {arrow_display}")
                
                previous_state = current_state

# Запуск потоку для відправки даних
sender_thread = threading.Thread(target=send_data, daemon=True)
sender_thread.start()

print("Керування: стрілки для руху, ESC для виходу")
print("Формат: [UP, DOWN, LEFT, RIGHT]")
print("Дані відправляються тільки при зміні стану\n")

# Запуск слухача клавіатури
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

ser.close()
print("\nЗ'єднання закрито.")