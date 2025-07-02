#!/usr/bin/env python3
"""
Target Tracking Emulator with WiFi and UART support
Enhanced version that supports both communication methods
"""

import tkinter as tk
from tkinter import ttk
import serial
import socket
import threading
import time
import math

class TargetEmulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Target Tracking Emulator - WiFi/UART")
        self.root.geometry("900x750")
        
        # Connection variables
        self.serial_port = None
        self.wifi_socket = None
        self.is_connected = False
        self.connection_type = "uart"  # "uart" or "wifi"
        self.auto_aim_active = False
        
        # Target coordinates
        self.target_x = 320
        self.target_y = 180
        self.crosshair_x = 320
        self.crosshair_y = 180
        
        # Frame dimensions (camera resolution)
        self.frame_width = 640
        self.frame_height = 360
        
        # Auto movement
        self.auto_move_active = False
        self.move_pattern = "circle"
        self.move_speed = 1.0
        self.move_angle = 0
        
        self.setup_ui()
        self.setup_bindings()
        
    def setup_ui(self):
        # Connection type selection frame
        conn_type_frame = ttk.LabelFrame(self.root, text="Connection Type", padding=10)
        conn_type_frame.pack(fill="x", padx=10, pady=5)
        
        self.conn_type_var = tk.StringVar(value="uart")
        ttk.Radiobutton(conn_type_frame, text="UART/Serial", variable=self.conn_type_var, 
                       value="uart", command=self.on_connection_type_change).pack(side="left", padx=10)
        ttk.Radiobutton(conn_type_frame, text="WiFi/TCP", variable=self.conn_type_var, 
                       value="wifi", command=self.on_connection_type_change).pack(side="left", padx=10)
        
        # Connection parameters frame
        conn_frame = ttk.LabelFrame(self.root, text="Connection Parameters", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        # UART settings
        self.uart_frame = ttk.Frame(conn_frame)
        self.uart_frame.grid(row=0, column=0, sticky="ew", columnspan=6)
        
        ttk.Label(self.uart_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value="/dev/ttyUSB0")
        self.port_entry = ttk.Entry(self.uart_frame, textvariable=self.port_var, width=15)
        self.port_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(self.uart_frame, text="Baud:").grid(row=0, column=2, sticky="w", padx=(20,0))
        self.baud_var = tk.StringVar(value="115200")
        self.baud_entry = ttk.Entry(self.uart_frame, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=0, column=3, padx=5)
        
        # WiFi settings
        self.wifi_frame = ttk.Frame(conn_frame)
        self.wifi_frame.grid(row=1, column=0, sticky="ew", columnspan=6)
        
        ttk.Label(self.wifi_frame, text="IP:").grid(row=0, column=0, sticky="w")
        self.ip_var = tk.StringVar(value="192.168.4.1")
        self.ip_entry = ttk.Entry(self.wifi_frame, textvariable=self.ip_var, width=15)
        self.ip_entry.grid(row=0, column=1, padx=5)
        
        ttk.Label(self.wifi_frame, text="Port:").grid(row=0, column=2, sticky="w", padx=(20,0))
        self.tcp_port_var = tk.StringVar(value="8080")
        self.tcp_port_entry = ttk.Entry(self.wifi_frame, textvariable=self.tcp_port_var, width=10)
        self.tcp_port_entry.grid(row=0, column=3, padx=5)
        
        # Connection controls
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=2, column=0, padx=10, pady=10, sticky="w")
        
        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=2, column=1, padx=10, pady=10, sticky="w")
        
        # Hide WiFi frame initially
        self.wifi_frame.grid_remove()
        
        # Control frame
        control_frame = ttk.LabelFrame(self.root, text="Target Control", padding=10)
        control_frame.pack(fill="x", padx=10, pady=5)
        
        # Manual coordinates input
        coord_input_frame = ttk.Frame(control_frame)
        coord_input_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0,10))
        
        ttk.Label(coord_input_frame, text="Target X:").grid(row=0, column=0, sticky="w")
        self.target_x_entry = ttk.Entry(coord_input_frame, width=8)
        self.target_x_entry.grid(row=0, column=1, padx=5)
        self.target_x_entry.insert(0, "320")
        
        ttk.Label(coord_input_frame, text="Y:").grid(row=0, column=2, sticky="w", padx=(20,0))
        self.target_y_entry = ttk.Entry(coord_input_frame, width=8)
        self.target_y_entry.grid(row=0, column=3, padx=5)
        self.target_y_entry.insert(0, "180")
        
        ttk.Button(coord_input_frame, text="Set Target", command=self.set_target_from_entry).grid(row=0, column=4, padx=10)
        
        ttk.Label(coord_input_frame, text="Cross X:").grid(row=1, column=0, sticky="w")
        self.cross_x_entry = ttk.Entry(coord_input_frame, width=8)
        self.cross_x_entry.grid(row=1, column=1, padx=5)
        self.cross_x_entry.insert(0, "320")
        
        ttk.Label(coord_input_frame, text="Y:").grid(row=1, column=2, sticky="w", padx=(20,0))
        self.cross_y_entry = ttk.Entry(coord_input_frame, width=8)
        self.cross_y_entry.grid(row=1, column=3, padx=5)
        self.cross_y_entry.insert(0, "180")
        
        ttk.Button(coord_input_frame, text="Set Cross", command=self.set_cross_from_entry).grid(row=1, column=4, padx=10)
        
        # Manual coordinates sliders
        ttk.Label(control_frame, text="Target X:").grid(row=1, column=0, sticky="w")
        self.target_x_var = tk.IntVar(value=320)
        self.target_x_scale = ttk.Scale(control_frame, from_=0, to=640, 
                                       variable=self.target_x_var, orient="horizontal",
                                       length=200, command=self.update_target_manual)
        self.target_x_scale.grid(row=1, column=1, padx=5)
        
        self.target_x_label = ttk.Label(control_frame, text="320")
        self.target_x_label.grid(row=1, column=2, padx=5)
        
        ttk.Label(control_frame, text="Target Y:").grid(row=2, column=0, sticky="w")
        self.target_y_var = tk.IntVar(value=180)
        self.target_y_scale = ttk.Scale(control_frame, from_=0, to=360,
                                       variable=self.target_y_var, orient="horizontal", 
                                       length=200, command=self.update_target_manual)
        self.target_y_scale.grid(row=2, column=1, padx=5)
        
        self.target_y_label = ttk.Label(control_frame, text="180")
        self.target_y_label.grid(row=2, column=2, padx=5)
        
        # Auto movement controls
        auto_frame = ttk.LabelFrame(control_frame, text="Auto Movement", padding=5)
        auto_frame.grid(row=3, column=0, columnspan=3, sticky="ew", pady=10)
        
        self.auto_move_var = tk.BooleanVar()
        self.auto_move_check = ttk.Checkbutton(auto_frame, text="Auto Move", 
                                              variable=self.auto_move_var,
                                              command=self.toggle_auto_move)
        self.auto_move_check.grid(row=0, column=0, sticky="w")
        
        ttk.Label(auto_frame, text="Pattern:").grid(row=0, column=1, padx=(20,5))
        self.pattern_var = tk.StringVar(value="circle")
        pattern_combo = ttk.Combobox(auto_frame, textvariable=self.pattern_var,
                                   values=["circle", "square", "figure8"], width=10)
        pattern_combo.grid(row=0, column=2, padx=5)
        
        ttk.Label(auto_frame, text="Speed:").grid(row=0, column=3, padx=(20,5))
        self.speed_var = tk.DoubleVar(value=1.0)
        speed_scale = ttk.Scale(auto_frame, from_=0.1, to=5.0,
                               variable=self.speed_var, orient="horizontal", length=100)
        speed_scale.grid(row=0, column=4, padx=5)
        
        # Auto aim control
        aim_frame = ttk.LabelFrame(self.root, text="Auto Aim Control", padding=10)
        aim_frame.pack(fill="x", padx=10, pady=5)
        
        self.start_btn = ttk.Button(aim_frame, text="Start Auto Aim (Space)", 
                                   command=self.start_auto_aim, state="disabled")
        self.start_btn.pack(side="left", padx=5)
        
        self.stop_btn = ttk.Button(aim_frame, text="Stop Auto Aim (Esc)", 
                                  command=self.stop_auto_aim, state="disabled")
        self.stop_btn.pack(side="left", padx=5)
        
        self.aim_status_label = ttk.Label(aim_frame, text="Auto Aim: OFF", foreground="red")
        self.aim_status_label.pack(side="left", padx=20)
        
        # Visual display
        display_frame = ttk.LabelFrame(self.root, text="Camera View (640x360)", padding=10)
        display_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.canvas = tk.Canvas(display_frame, width=640, height=360, bg="black")
        self.canvas.pack()
        
        # Info display
        info_frame = ttk.LabelFrame(self.root, text="Connection Info", padding=5)
        info_frame.pack(fill="x", padx=10, pady=5)
        
        self.info_text = tk.Text(info_frame, height=4, width=80)
        self.info_text.pack(side="left", fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(info_frame, orient="vertical", command=self.info_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.info_text.config(yscrollcommand=scrollbar.set)
        
        self.update_display()
        
    def setup_bindings(self):
        self.root.bind("<Key>", self.on_key_press)
        self.root.focus_set()
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        
    def on_connection_type_change(self):
        """Handle connection type change"""
        self.connection_type = self.conn_type_var.get()
        
        if self.connection_type == "uart":
            self.uart_frame.grid()
            self.wifi_frame.grid_remove()
        else:
            self.wifi_frame.grid()
            self.uart_frame.grid_remove()
            
        # Disconnect if connected
        if self.is_connected:
            self.disconnect()
            
    def connect_uart(self):
        """Connect via UART/Serial"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_var.get(),
                baudrate=int(self.baud_var.get()),
                timeout=0.1
            )
            self.is_connected = True
            self.log(f"UART connected to {self.port_var.get()}")
            return True
        except Exception as e:
            self.log(f"UART connection failed: {e}")
            return False
            
    def connect_wifi(self):
        """Connect via WiFi/TCP"""
        try:
            self.wifi_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.wifi_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.wifi_socket.settimeout(5.0)
            self.wifi_socket.connect((self.ip_var.get(), int(self.tcp_port_var.get())))
            self.is_connected = True
            self.log(f"WiFi connected to {self.ip_var.get()}:{self.tcp_port_var.get()}")
            return True
        except Exception as e:
            self.log(f"WiFi connection failed: {e}")
            return False
            
    def disconnect(self):
        """Disconnect current connection"""
        if self.connection_type == "uart" and self.serial_port:
            try:
                self.serial_port.close()
                self.serial_port = None
            except:
                pass
        elif self.connection_type == "wifi" and self.wifi_socket:
            try:
                self.wifi_socket.close()
                self.wifi_socket = None
            except:
                pass
                
        self.is_connected = False
        self.log("Disconnected")
        
    def toggle_connection(self):
        """Toggle connection based on selected type"""
        if not self.is_connected:
            success = False
            if self.connection_type == "uart":
                success = self.connect_uart()
            else:
                success = self.connect_wifi()
                
            if success:
                self.status_label.config(text=f"Connected ({self.connection_type.upper()})", foreground="green")
                self.connect_btn.config(text="Disconnect")
                self.start_btn.config(state="normal")
                self.stop_btn.config(state="normal")
        else:
            self.disconnect()
            self.status_label.config(text="Disconnected", foreground="red")
            self.connect_btn.config(text="Connect")
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="disabled")
            self.stop_auto_aim()
            
    def send_command(self, command):
        """Send command via current connection type"""
        if not self.is_connected:
            return
            
        try:
            if self.connection_type == "uart" and self.serial_port:
                self.serial_port.write(f"{command}\n".encode())
                self.log(f"UART -> {command}")
            elif self.connection_type == "wifi" and self.wifi_socket:
                self.wifi_socket.sendall(f"{command}\n".encode())
                self.log(f"WiFi -> {command}")
        except Exception as e:
            self.log(f"Send error ({self.connection_type}): {e}")
            self.is_connected = False
            
    def on_key_press(self, event):
        if event.keysym == "space":
            self.start_auto_aim()
        elif event.keysym == "Escape":
            self.stop_auto_aim()
        elif event.keysym in ["Up", "Down", "Left", "Right"]:
            self.move_target_with_keys(event.keysym)
            
    def move_target_with_keys(self, key):
        step = 10
        if key == "Up" and self.target_y > step:
            self.target_y -= step
        elif key == "Down" and self.target_y < self.frame_height - step:
            self.target_y += step
        elif key == "Left" and self.target_x > step:
            self.target_x -= step
        elif key == "Right" and self.target_x < self.frame_width - step:
            self.target_x += step
            
        self.target_x_var.set(self.target_x)
        self.target_y_var.set(self.target_y)
        self.update_display()
        self.send_target_coordinates()
        
    def on_canvas_click(self, event):
        self.target_x = max(0, min(self.frame_width, event.x))
        self.target_y = max(0, min(self.frame_height, event.y))
        self.target_x_var.set(self.target_x)
        self.target_y_var.set(self.target_y)
        self.update_display()
        self.send_target_coordinates()
        
    def on_canvas_right_click(self, event):
        self.crosshair_x = max(0, min(self.frame_width, event.x))
        self.crosshair_y = max(0, min(self.frame_height, event.y))
        self.cross_x_entry.delete(0, tk.END)
        self.cross_x_entry.insert(0, str(int(self.crosshair_x)))
        self.cross_y_entry.delete(0, tk.END)
        self.cross_y_entry.insert(0, str(int(self.crosshair_y)))
        self.update_display()
        self.send_crosshair_coordinates()
        
    def set_target_from_entry(self):
        try:
            x = int(self.target_x_entry.get())
            y = int(self.target_y_entry.get())
            x = max(0, min(self.frame_width, x))
            y = max(0, min(self.frame_height, y))
            
            self.target_x = x
            self.target_y = y
            self.target_x_var.set(x)
            self.target_y_var.set(y)
            self.target_x_label.config(text=str(x))
            self.target_y_label.config(text=str(y))
            
            self.update_display()
            self.send_target_coordinates()
            self.log(f"Target set to ({x}, {y})")
        except ValueError:
            self.log("Invalid target coordinates!")
            
    def set_cross_from_entry(self):
        try:
            x = int(self.cross_x_entry.get())
            y = int(self.cross_y_entry.get())
            x = max(0, min(self.frame_width, x))
            y = max(0, min(self.frame_height, y))
            
            self.crosshair_x = x
            self.crosshair_y = y
            
            self.update_display()
            self.send_crosshair_coordinates()
            self.log(f"Crosshair set to ({x}, {y})")
        except ValueError:
            self.log("Invalid crosshair coordinates!")
        
    def update_target_manual(self, value=None):
        self.target_x = self.target_x_var.get()
        self.target_y = self.target_y_var.get()
        self.target_x_label.config(text=str(self.target_x))
        self.target_y_label.config(text=str(self.target_y))
        # Update entry fields
        self.target_x_entry.delete(0, tk.END)
        self.target_x_entry.insert(0, str(self.target_x))
        self.target_y_entry.delete(0, tk.END)
        self.target_y_entry.insert(0, str(self.target_y))
        self.update_display()
        if self.auto_aim_active:
            self.send_target_coordinates()
            
    def toggle_auto_move(self):
        self.auto_move_active = self.auto_move_var.get()
        if self.auto_move_active:
            self.start_auto_movement()
            
    def start_auto_movement(self):
        if not self.auto_move_active:
            return
            
        # Calculate new position based on pattern
        pattern = self.pattern_var.get()
        speed = self.speed_var.get()
        
        center_x, center_y = self.frame_width // 2, self.frame_height // 2
        radius = 80
        
        if pattern == "circle":
            self.target_x = center_x + radius * math.cos(self.move_angle)
            self.target_y = center_y + radius * math.sin(self.move_angle)
        elif pattern == "square":
            # Square pattern
            side = radius * 2
            t = (self.move_angle % (4 * math.pi)) / math.pi
            if t < 1:  # Top side
                self.target_x = center_x - side//2 + side * t
                self.target_y = center_y - side//2
            elif t < 2:  # Right side
                self.target_x = center_x + side//2
                self.target_y = center_y - side//2 + side * (t-1)
            elif t < 3:  # Bottom side
                self.target_x = center_x + side//2 - side * (t-2)
                self.target_y = center_y + side//2
            else:  # Left side
                self.target_x = center_x - side//2
                self.target_y = center_y + side//2 - side * (t-3)
        elif pattern == "figure8":
            self.target_x = center_x + radius * math.sin(self.move_angle)
            self.target_y = center_y + radius * math.sin(self.move_angle * 2) / 2
            
        # Keep within bounds
        self.target_x = max(10, min(self.frame_width - 10, self.target_x))
        self.target_y = max(10, min(self.frame_height - 10, self.target_y))
        
        self.move_angle += 0.05 * speed
        
        self.target_x_var.set(int(self.target_x))
        self.target_y_var.set(int(self.target_y))
        self.update_display()
        
        if self.auto_aim_active:
            self.send_target_coordinates()
            
        if self.auto_move_active:
            self.root.after(50, self.start_auto_movement)
            
    def start_auto_aim(self):
        if not self.is_connected:
            self.log("Not connected!")
            return
            
        self.auto_aim_active = True
        self.aim_status_label.config(text="Auto Aim: ON", foreground="green")
        self.send_command("AA_SYS 1")
        self.log("Auto aim started")
        
        # Start sending coordinates periodically
        self.send_coordinates_loop()
        
    def stop_auto_aim(self):
        self.auto_aim_active = False
        self.aim_status_label.config(text="Auto Aim: OFF", foreground="red")
        if self.is_connected:
            self.send_command("AA_SYS 0")
        self.log("Auto aim stopped")
        
    def send_coordinates_loop(self):
        if self.auto_aim_active and self.is_connected:
            self.send_target_coordinates()
            self.root.after(100, self.send_coordinates_loop)  # Send every 100ms
            
    def send_target_coordinates(self):
        if self.is_connected and self.auto_aim_active:
            command = f"TARGET {int(self.target_x)} {int(self.target_y)}"
            self.send_command(command)
                
    def send_crosshair_coordinates(self):
        if self.is_connected:
            command = f"CROSS {int(self.crosshair_x)} {int(self.crosshair_y)}"
            self.send_command(command)
                
    def update_display(self):
        self.canvas.delete("all")
        
        # Draw camera frame
        self.canvas.create_rectangle(0, 0, self.frame_width, self.frame_height, 
                                   outline="gray", width=2)
        
        # Draw crosshair
        cx, cy = self.crosshair_x, self.crosshair_y
        self.canvas.create_line(cx-20, cy, cx+20, cy, fill="white", width=2)
        self.canvas.create_line(cx, cy-20, cx, cy+20, fill="white", width=2)
        self.canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill="white")
        
        # Draw target
        tx, ty = self.target_x, self.target_y
        self.canvas.create_oval(tx-15, ty-15, tx+15, ty+15, 
                              outline="red", fill="red", width=3)
        
        # Draw error line
        self.canvas.create_line(cx, cy, tx, ty, fill="yellow", width=2, dash=(5,5))
        
        # Draw error values
        error_x = tx - cx
        error_y = ty - cy
        distance = math.sqrt(error_x**2 + error_y**2)
        
        self.canvas.create_text(10, 10, anchor="nw", fill="white",
                              text=f"Target: ({int(tx)}, {int(ty)})")
        self.canvas.create_text(10, 30, anchor="nw", fill="white",
                              text=f"Crosshair: ({int(cx)}, {int(cy)})")
        self.canvas.create_text(10, 50, anchor="nw", fill="white",
                              text=f"Error: ({int(error_x)}, {int(error_y)})")
        self.canvas.create_text(10, 70, anchor="nw", fill="white",
                              text=f"Distance: {distance:.1f}")
        
        # Connection info
        conn_info = f"Connection: {self.connection_type.upper()}"
        if self.is_connected:
            if self.connection_type == "uart":
                conn_info += f" ({self.port_var.get()})"
            else:
                conn_info += f" ({self.ip_var.get()}:{self.tcp_port_var.get()})"
        else:
            conn_info += " (Not connected)"
            
        self.canvas.create_text(10, 90, anchor="nw", fill="cyan", text=conn_info)
        
        # Controls help
        self.canvas.create_text(self.frame_width - 10, 10, anchor="ne", fill="cyan",
                              text="Left Click: Move target")
        self.canvas.create_text(self.frame_width - 10, 30, anchor="ne", fill="cyan",
                              text="Right Click: Set crosshair")
        self.canvas.create_text(self.frame_width - 10, 50, anchor="ne", fill="cyan",
                              text="Space: Start auto aim")
        self.canvas.create_text(self.frame_width - 10, 70, anchor="ne", fill="cyan",
                              text="Esc: Stop auto aim")
        
    def log(self, message):
        timestamp = time.strftime('%H:%M:%S')
        self.info_text.insert(tk.END, f"{timestamp} - {message}\n")
        self.info_text.see(tk.END)
        
    def __del__(self):
        """Cleanup on destruction"""
        if self.is_connected:
            self.disconnect()
        
if __name__ == "__main__":
    root = tk.Tk()
    app = TargetEmulator(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        # Ensure cleanup
        if hasattr(app, 'is_connected') and app.is_connected:
            app.disconnect()
