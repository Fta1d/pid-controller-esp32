#!/usr/bin/env python3
"""
ESP32 Turret Control System
Supports both keyboard and gamepad input with optimized command transmission
"""

import socket
import pygame
from pynput import keyboard
import threading
import time
import sys

ESP32_IP = "192.168.4.1"
ESP32_PORT = 8080

class TurretController:
    def __init__(self):
        self.sock = None  # Just TCP socket like original
        self.running = True
        self.gamepad_mode = False
        self.gamepad = None
        self.esp32_connected = False
        
        # Keyboard state
        self.key_states = {'up': False, 'down': False, 'left': False, 'right': False}
        
    def connect_to_esp32(self):
        """Connect to ESP32 with optimized settings"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.connect((ESP32_IP, ESP32_PORT))
            self.esp32_connected = True
            print("ESP32 connected successfully")
            return True
        except Exception as e:
            print(f"ESP32 connection failed: {e}")
            print("Running in test mode")
            self.esp32_connected = False
            return False
    
    def init_gamepad(self):
        """Initialize gamepad"""
        try:
            pygame.init()
            pygame.joystick.init()
            
            count = pygame.joystick.get_count()
            print(f"Gamepads found: {count}")
            
            if count == 0:
                print("No gamepad available")
                return False
                
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            
            print(f"Gamepad: {self.gamepad.get_name()}")
            print(f"Axes: {self.gamepad.get_numaxes()}, Buttons: {self.gamepad.get_numbuttons()}")
            
            return True
            
        except Exception as e:
            print(f"Gamepad initialization error: {e}")
            return False
    
    def send_to_esp32(self, command):
        """Send command to ESP32 with minimal latency"""
        try:
            if self.esp32_connected and self.sock:
                data = (command + '\n').encode()
                self.sock.sendall(data)
                print(f"-> ESP32: {command}")
            else:
                print(f"-> (test): {command}")
        except Exception as e:
            print(f"Transmission error: {e}")
            self.esp32_connected = False
    
    def keyboard_mode(self):
        """Keyboard control mode (matching original logic)"""
        print("KEYBOARD MODE")
        print("Arrow keys - movement, SPACE - shoot, TAB/G - gamepad, ESC - exit")
        
        last_state = ""
        
        while self.running and not self.gamepad_mode:
            # Form 4-character state string like original: ABCD format
            current_state = (
                ('A' if self.key_states['up'] else '0') +
                ('B' if self.key_states['down'] else '0') + 
                ('D' if self.key_states['left'] else '0') +
                ('C' if self.key_states['right'] else '0')
            )
            
            # Send ONLY when state changes
            if current_state != last_state:
                self.send_to_esp32(current_state)
                last_state = current_state
            
            time.sleep(0.03)  # Faster polling for better response
    
    def gamepad_mode_loop(self):
        """Gamepad control mode (optimized for minimal latency)"""
        print("GAMEPAD MODE")
        print("Left stick - movement, button 0 - shoot, button 1 - return to keyboard")
        
        deadzone = 0.2
        last_command = ""
        last_send_time = 0
        send_interval = 0.05  # Send max every 50ms (20 FPS)
        
        while self.running and self.gamepad_mode:
            try:
                # Process events (buttons only)
                events = pygame.event.get()
                for event in events:
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 2:  # Shoot
                            self.send_to_esp32("SHOOT")
                        elif event.button == 1:  # Return to keyboard
                            print("Returning to keyboard mode")
                            self.send_to_esp32("STOP")
                            self.gamepad_mode = False
                            return
                
                pygame.event.pump()
                
                # Read sticks (rate limited)
                current_time = time.time()
                if current_time - last_send_time >= send_interval:
                    current_command = ""
                    if self.gamepad:
                        x = self.gamepad.get_axis(0)  # Left stick X
                        y = self.gamepad.get_axis(1)  # Left stick Y
                        
                        # Form movement command
                        commands = []
                        
                        # Horizontal movement
                        if abs(x) > deadzone:
                            if x < 0:
                                speed = max(1, min(255, int(abs(x) * 255)))
                                commands.append(f"R{speed:03d}")
                            else:
                                speed = max(1, min(255, int(x * 255)))
                                commands.append(f"L{speed:03d}")
                        
                        # Vertical movement
                        if abs(y) > deadzone:
                            if y < 0:
                                speed = max(1, min(255, int(abs(y) * 255)))
                                commands.append(f"D{speed:03d}")
                            else:
                                speed = max(1, min(255, int(y * 255)))
                                commands.append(f"U{speed:03d}")
                        
                        # Form command
                        if commands:
                            current_command = ",".join(commands)
                        else:
                            current_command = "STOP"
                    
                    # Send ONLY when command changes (no periodic resends!)
                    if current_command != last_command:
                        self.send_to_esp32(current_command)
                        last_command = current_command
                        last_send_time = current_time
                
                time.sleep(0.01)  # Fast polling but rate-limited sending
                
            except Exception as e:
                print(f"Gamepad error: {e}")
                self.gamepad_mode = False
                break
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            if key == keyboard.Key.esc:
                print("Shutting down")
                self.running = False
                return False
            
            # Mode switching
            if key == keyboard.Key.tab or (hasattr(key, 'char') and key.char == 'g'):
                if self.gamepad and not self.gamepad_mode:
                    print("Switching to gamepad mode")
                    self.gamepad_mode = True
                else:
                    print("Gamepad unavailable or already active")
                return
            
            # Only in keyboard mode
            if not self.gamepad_mode:
                if key == keyboard.Key.up:
                    self.key_states['up'] = True
                elif key == keyboard.Key.down:
                    self.key_states['down'] = True
                elif key == keyboard.Key.left:
                    self.key_states['left'] = True
                elif key == keyboard.Key.right:
                    self.key_states['right'] = True
                elif key == keyboard.Key.space:
                    self.send_to_esp32(" ")
                elif hasattr(key, 'char') and key.char and key.char.isprintable():
                    self.send_to_esp32(key.char)
                    
        except AttributeError:
            pass
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            if not self.gamepad_mode:
                if key == keyboard.Key.up:
                    self.key_states['up'] = False
                elif key == keyboard.Key.down:
                    self.key_states['down'] = False
                elif key == keyboard.Key.left:
                    self.key_states['left'] = False
                elif key == keyboard.Key.right:
                    self.key_states['right'] = False
                    
        except AttributeError:
            pass
    
    def run(self):
        """Main program loop"""
        print("TURRET CONTROL SYSTEM")
        print("=" * 40)
        
        # Connection
        self.connect_to_esp32()
        gamepad_available = self.init_gamepad()
        
        print("\nCONTROLS:")
        print("Keyboard: Arrow keys - movement, SPACE - shoot")
        if gamepad_available:
            print("Gamepad: TAB/G - switch to gamepad")
            print("Gamepad: sticks - movement, button 0 - shoot, button 1 - back")
        print("ESC - exit")
        print("=" * 40)
        
        # Start keyboard mode thread
        keyboard_thread = threading.Thread(target=self.keyboard_mode, daemon=True)
        keyboard_thread.start()
        
        # Keyboard listener in main thread
        try:
            with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                while self.running:
                    if self.gamepad_mode and gamepad_available:
                        self.gamepad_mode_loop()
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\nInterrupted")
        
        # Cleanup
        self.running = False
        print("Shutting down...")
        
        # Send final STOP
        if self.esp32_connected:
            try:
                self.send_to_esp32("STOP")
                time.sleep(0.1)
            except:
                pass
        
        # Close connection
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
            except:
                pass
        
        # Cleanup pygame
        if self.gamepad:
            try:
                pygame.quit()
            except:
                pass
        
        print("Program terminated")

def main():
    """Main function"""
    print("Starting turret control system...")
    
    # Check dependencies
    try:
        import pygame
        import pynput
    except ImportError as e:
        print(f"Missing dependency: {e}")
        print("Install with: pip install pygame pynput")
        sys.exit(1)
    
    controller = TurretController()
    controller.run()

if __name__ == "__main__":
    main()