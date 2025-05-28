import socket
from pynput import keyboard
import threading
import time

# Замість serial підключення - TCP
ESP32_IP = "192.168.4.1"  # IP адреса вашого ESP32
ESP32_PORT = 8080

# Створити TCP з'єднання
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def connect_to_esp32():
    """Підключення до ESP32"""
    try:
        sock.connect((ESP32_IP, ESP32_PORT))
        print(f"Підключено до ESP32: {ESP32_IP}:{ESP32_PORT}")
        return True
    except Exception as e:
        print(f"Помилка підключення: {e}")
        return False

# Решта коду залишається майже такий же!
key_states = {
    'up': '0',
    'down': '0',
    'left': '0',
    'right': '0'
}

previous_state = ''
lock = threading.Lock()
running = True
state_changed = threading.Event()

def get_current_state():
    return key_states['up'] + key_states['down'] + key_states['left'] + key_states['right']

def send_single_char(char):
    """Відправляє одиночний символ через WiFi"""
    try:
        message = str(char) + '\n'
        sock.send(message.encode())  # Замість ser.write()
        timestamp = time.strftime('%H:%M:%S')
        display_char = "SPACE" if char == ' ' else char
        print(f"[{timestamp}] Символ: {display_char}")
    except Exception as e:
        print(f"Помилка відправки: {e}")

# on_press та on_release залишаються БЕЗ ЗМІН!
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
            elif key == keyboard.Key.space:
                send_single_char(' ')
                return
            elif hasattr(key, 'char') and key.char and key.char.isprintable():
                send_single_char(key.char)
                return
            
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
        state_changed.wait()
        state_changed.clear()
        
        if not running:
            break
            
        with lock:
            current_state = get_current_state()
            
            if current_state != previous_state:
                try:
                    message = current_state + '\n'
                    sock.send(message.encode())  # Замість ser.write()
                    
                    # Візуалізація залишається така сама
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
                except Exception as e:
                    print(f"Помилка відправки: {e}")

# Запуск
if connect_to_esp32():
    sender_thread = threading.Thread(target=send_data, daemon=True)
    sender_thread.start()
    
    print("Керування через WiFi:")
    print("- Стрілки для руху")
    print("- Всі символи та ПРОБІЛ для команд")
    print("- ESC для виходу\n")
    
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    
    sock.close()
    print("\nWiFi з'єднання закрито.")
else:
    print("Не вдалося підключитися до ESP32")

# import serial
# from pynput import keyboard
# import threading
# import time

# # Налаштування порту (змініть на ваш)
# ser = serial.Serial('/dev/ttyUSB0', 115200)

# # Словник для зберігання стану кожної клавіші
# key_states = {
#     'up': '0',
#     'down': '0',
#     'left': '0',
#     'right': '0'
# }

# # Змінна для збереження попереднього стану
# previous_state = ''
# lock = threading.Lock()
# running = True
# state_changed = threading.Event()

# def get_current_state():
#     """Повертає поточний стан всіх клавіш як рядок"""
#     return key_states['up'] + key_states['down'] + key_states['left'] + key_states['right']

# def send_single_char(char):
#     """Відправляє одиночний символ негайно"""
#     message = str(char) + '\n'
#     ser.write(message.encode())
#     timestamp = time.strftime('%H:%M:%S')
    
#     # Спеціальне відображення для пробілу
#     display_char = "SPACE" if char == ' ' else char
#     print(f"[{timestamp}] Символ: {display_char}")

# def on_press(key):
#     try:
#         with lock:
#             changed = False
            
#             # Обробка стрілок
#             if key == keyboard.Key.up and key_states['up'] != 'A':
#                 key_states['up'] = 'A'
#                 changed = True
#             elif key == keyboard.Key.down and key_states['down'] != 'B':
#                 key_states['down'] = 'B'
#                 changed = True
#             elif key == keyboard.Key.left and key_states['left'] != 'D':
#                 key_states['left'] = 'D'
#                 changed = True
#             elif key == keyboard.Key.right and key_states['right'] != 'C':
#                 key_states['right'] = 'C'
#                 changed = True
            
#             # Обробка пробілу як спеціальної клавіші
#             elif key == keyboard.Key.space:
#                 send_single_char(' ')
#                 return  # Не встановлюємо state_changed для символів
            
#             # Обробка всіх символів (літери, цифри, знаки пунктуації)
#             elif hasattr(key, 'char') and key.char and key.char.isprintable():
#                 send_single_char(key.char)
#                 return  # Не встановлюємо state_changed для символів
            
#             if changed:
#                 state_changed.set()
                
#     except AttributeError:
#         pass

# def on_release(key):
#     global running
#     try:
#         with lock:
#             changed = False
            
#             if key == keyboard.Key.up and key_states['up'] != '0':
#                 key_states['up'] = '0'
#                 changed = True
#             elif key == keyboard.Key.down and key_states['down'] != '0':
#                 key_states['down'] = '0'
#                 changed = True
#             elif key == keyboard.Key.left and key_states['left'] != '0':
#                 key_states['left'] = '0'
#                 changed = True
#             elif key == keyboard.Key.right and key_states['right'] != '0':
#                 key_states['right'] = '0'
#                 changed = True
#             elif key == keyboard.Key.esc:
#                 running = False
#                 state_changed.set()
#                 return False
            
#             if changed:
#                 state_changed.set()
                
#     except AttributeError:
#         pass

# def send_data():
#     global previous_state
#     while running:
#         # Чекаємо на зміну стану
#         state_changed.wait()
#         state_changed.clear()
        
#         if not running:
#             break
            
#         with lock:
#             current_state = get_current_state()
            
#             # Відправляємо тільки якщо стан змінився
#             if current_state != previous_state:
#                 message = current_state + '\n'
#                 ser.write(message.encode())
                
#                 # Візуалізація
#                 display = message.strip().replace('0', '_')
#                 arrows = []
#                 if key_states['up'] == 'A': arrows.append('↑')
#                 if key_states['down'] == 'B': arrows.append('↓')
#                 if key_states['left'] == 'D': arrows.append('←')
#                 if key_states['right'] == 'C': arrows.append('→')
                
#                 arrow_display = ' '.join(arrows) if arrows else 'нічого'
#                 timestamp = time.strftime('%H:%M:%S')
#                 print(f"[{timestamp}] Зміна стану: [{display}] - {arrow_display}")
                
#                 previous_state = current_state

# # Запуск потоку для відправки даних
# sender_thread = threading.Thread(target=send_data, daemon=True)
# sender_thread.start()

# print("Керування:")
# print("- Стрілки для руху (відправляються при зміні стану)")
# print("- ВСІ символи (літери, цифри, знаки) та ПРОБІЛ для команд (відправляються негайно)")
# print("- ESC для виходу")
# print("Формат стрілок: [UP, DOWN, LEFT, RIGHT]")
# print("Дані відправляються тільки при зміни стану\n")

# # Запуск слухача клавіатури
# with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
#     listener.join()

# ser.close()
# print("\nЗ'єднання закрито.")